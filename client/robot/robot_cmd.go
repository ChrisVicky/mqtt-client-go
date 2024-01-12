package robot

import (
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

	cmd := exec.Command(command)
	r.cmdRecords[id] = cmd

	// Async
	go func() {
		err := cmd.Run()
		if err != nil {
			logger.Errorf("Error Execution: %v (%+v)\n", err, id)
		}
		r.cmdsMu.Lock()
		delete(r.cmdRecords, id)
		r.cmdsMu.Unlock()
	}()

	logger.Infof("Run cmd: %+v:%v", id, command)

	return nil
}

// Run Cmd Sync
func (r *rcm) RunCmd(id cmd.CMDEnum) error {
	command, ok := cmdMap[id]
	if !ok {
		return fmt.Errorf("no command id: %+v", id)
	}

	cmd := exec.Command(command)

	r.cmdsMu.Lock()
	r.cmdRecords[id] = cmd
	r.cmdsMu.Unlock()

	err := cmd.Run()

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
	for k, v := range r.cmdRecords {
		logger.Infof("Killing %+v: %v", k, v)
		if err := r.StopCmd(k); err != nil {
			logger.Error(err)
		}
	}
}

// Deprecated
//
// func (r *Robot) run(cmd_str string, uselog bool) {
// 	cmd := exec.Command(cmd_str)
//
// 	logger.Infof("run cmd: %v", cmd.String())
//
// 	var wg sync.WaitGroup
// 	if uselog {
// 		stdout, _ := cmd.StdoutPipe()
// 		wg.Add(1)
// 		go func() {
// 			defer wg.Done()
// 			escanner := bufio.NewScanner(stdout)
// 			for escanner.Scan() {
// 				line := escanner.Text()
// 				r.loginfo(line)
// 			}
// 		}()
// 	}
//
// 	cmd.Run()
// 	wg.Wait()
// }
