package robot

import (
	"bufio"
	"fmt"
	"mqttclient/client/cmd"
	"mqttclient/logger"
	"os/exec"
	"sync"
)

const (
	MAIN_NODE cmd.CMDEnum = iota
	G_MAPPING
	PS2
	BUILD_MAP
)

var cmdMap = map[cmd.CMDEnum]string{
	MAIN_NODE: "roslaunch huanyu_robot_start Huanyu_robot_start.launch ", // main node
	G_MAPPING: "roslaunch huanyu_robot_start gmapping_slam.launch ",      // gmapping
	PS2:       "roslaunch huanyu_joy huanyu_ps2_control.launch ",         // Ps2
	BUILD_MAP: fmt.Sprintf("rosrun map_server map_saver -f %s", mapName), // buildMap
}

// Robot Command Manager
type rcm struct {
	cmdsMu     *sync.Mutex               // Cmds Mutex for Records
	cmdRecords map[cmd.CMDEnum]*exec.Cmd // Cmds Record
}

// Launch Cmd Async
func (r *rcm) RunCmdAsync(id cmd.CMDEnum) error {
	r.cmdsMu.Lock()
	defer r.cmdsMu.Unlock()

	command, ok := cmdMap[id]
	if !ok {
		return fmt.Errorf("no command id: %+v", id)
	}

	cmd := exec.Command("bash", "-c", command)
	r.cmdRecords[id] = cmd

	// Async
	go func() {
		cmd_str := cmd.String()
		stderr, _ := cmd.StderrPipe()
		err := cmd.Start()
		logger.Infof("[Async] %+v:%v Launched", id, cmd_str)
		if err != nil {
			logger.Errorf("%v: %v", err, cmd_str)
		}
		scanner := bufio.NewScanner(stderr)
		for scanner.Scan() {
			logger.Tracef("[%v] %v", cmd_str, scanner.Text())
		}
		logger.Infof("[Async] %+v:%v Finished", id, cmd_str)
		r.cmdsMu.Lock()
		delete(r.cmdRecords, id)
		r.cmdsMu.Unlock()
	}()

	return nil
}

// Run Cmd Sync
func (r *rcm) RunCmd(id cmd.CMDEnum) error {
	command, ok := cmdMap[id]
	if !ok {
		return fmt.Errorf("no command id: %+v", id)
	}

	cmd := exec.Command("bash", "-c", command)

	r.cmdsMu.Lock()
	r.cmdRecords[id] = cmd
	r.cmdsMu.Unlock()

	logger.Infof("[Sync] %+v:%v Run", id, cmd.String())
	err := cmd.Run()
	logger.Infof("[Sync] %+v:%v Done", id, cmd.String())

	r.cmdsMu.Lock()
	delete(r.cmdRecords, id)
	r.cmdsMu.Unlock()

	return err
}

func (r *rcm) StopCmd(id cmd.CMDEnum) (err error) {
	r.cmdsMu.Lock()
	defer r.cmdsMu.Unlock()
	err = r.cmdRecords[id].Process.Kill()
	if err == nil {
		delete(r.cmdRecords, id)
		logger.Infof("%+v, Killed", id)
	}
	return err
}

func (r *rcm) CleanCmds() {
	logger.Infof("Clean All Cmds Start")
	for k, v := range r.cmdRecords {
		logger.Infof("Killing %+v: %v", k, v)
		if err := r.StopCmd(k); err != nil {
			logger.Error(err)
		}
	}
	logger.Infof("Clean All Cmds Done")
}
