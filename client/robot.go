package client

import (
	"bufio"
	"fmt"
	"mqttclient/logger"
	"mqttclient/reply"
	"os"
	"os/exec"
	"sync"
	"time"

	"github.com/BurntSushi/toml"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

const (
	mapName           = "map"
	defaultConfigName = "robot.toml"
)

type Robot struct {
	mqtt.Client
	Config     string
	DeviceId   string `toml:"deviceId"`
	DeviceType string `toml:"devType"`
	stat       string // on / off
	params     string
}

func (r *Robot) baseApi() string {
	return fmt.Sprintf("/broker/%s/%s", r.DeviceType, r.DeviceId)
}

func (r *Robot) loadConfig() (err error) {
	var cBytes []byte
	fn := r.Config
	if _, err = os.Stat(fn); err != nil {
		fn = defaultConfigName
	}
	if cBytes, err = os.ReadFile(fn); err != nil {
		return
	}
	if err = toml.Unmarshal(cBytes, r); err != nil {
		return
	}
	return
}

func (r *Robot) parseInform() map[string]string {
	inform := map[string]string{
		"deviceId": r.DeviceId,
		"devType":  r.DeviceType,
		"stat":     r.stat,
		"param":    "",
	}
	return inform
}

func (r *Robot) Running() bool {
	return r.stat == "on"
}

func (r *Robot) Online() {
	if err := r.loadConfig(); err != nil {
		logger.Fatal(err)
	}

	logger.Info("Online")

	r.subRegistration()

	r.stat = "on"
	tk := r.Publish("/client/online", 1, false, reply.Ok(r.parseInform()))
	tk.Wait()
}

func (r *Robot) Offline() {
	r.stat = "off"
	tk := r.Publish("/client/offline", 1, false, reply.Ok(r.parseInform()))
	tk.Wait()
}

func (r *Robot) pubStart() {
	tk := r.Publish(r.baseApi()+"/start", 1, false, "")
	tk.Wait()
}

// Register Subscription
func (r *Robot) subRegistration() {
	subscriptions := map[string]func(mqtt.Client, mqtt.Message){
		// [sub] base/map/start
		r.baseApi() + "/map/start": func(mqtt.Client, mqtt.Message) {
			r.pubStart()

			go r.mapGeneration()
		},

		// [sub] base/map/fetch
		r.baseApi() + "/map/fetch": func(mqtt.Client, mqtt.Message) {
			r.pubStart()

			r.buildMap()
			// Read map from disk & sent via publish
			var payload []byte
			if f, err := os.ReadFile(mapName); err != nil {
				// Error Emit
				payload = []byte(fmt.Sprintf("Wrong: %v", err))
			} else {
				payload = f
			}
			// This Do not need to wait
			r.Publish(r.baseApi()+"/map/png", 0, false, payload)
		},

		// [sub] byte
		r.baseApi() + "/bye": func(mqtt.Client, mqtt.Message) {
			r.stat = "off"
		},

		// [sub] test
		r.baseApi() + "/test": func(mqtt.Client, mqtt.Message) {
			logger.Infof("Testing Called")
		},
	}

	for k, v := range subscriptions {
		logger.Infof("Sub: %s", k)
		tk := r.Subscribe(k, 1, v)
		tk.Wait()
	}
}

func (r *Robot) loginfo(str string) {
	tk := r.Publish(r.baseApi()+"/info", 0, false, str)
	tk.Wait()
}

func (r *Robot) logerror(str string) {
	tk := r.Publish(r.baseApi()+"/error", 0, false, str)
	tk.Wait()
}

func (r *Robot) run(cmd_str string, uselog bool) {
	cmd := exec.Command(cmd_str)

	logger.Infof("run cmd: %v", cmd.String())

	var wg sync.WaitGroup
	if uselog {
		stdout, _ := cmd.StdoutPipe()
		wg.Add(1)
		go func() {
			defer wg.Done()
			escanner := bufio.NewScanner(stdout)
			for escanner.Scan() {
				line := escanner.Text()
				r.loginfo(line)
			}
		}()
	}

	cmd.Run()
	wg.Wait()
}

// NOTE: Run multiple cmds simultaniously
// this can block (wg.Wait())
// Thus when running ROS commands, you shall use `go runCmds(cmds, uselog)`
func (r *Robot) runCmds(cmds []string, uselog bool) {
	var wg sync.WaitGroup
	for _, cmd := range cmds {
		wg.Add(1)
		go func() {
			defer wg.Done()
			r.run(cmd, uselog)
		}()
		time.Sleep(time.Second)
	}
	wg.Wait()
}

// TODO: These Commands shall be LOADED via a Configuration
func (r *Robot) mapGeneration() {
	cmds := []string{
		"roslaunch huanyu_robot_start Huanyu_robot_start.launch ", // main node
		"roslaunch huanyu_robot_start gmapping_slam.launch ",      // gmapping
		"roslaunch huanyu_joy huanyu_ps2_control.launch ",         // Ps2
	}
	r.runCmds(cmds, false)
}

func (r *Robot) buildMap() {
	cmds := []string{
		fmt.Sprintf("rosrun map_server map_saver -f %s", mapName),
	}
	r.runCmds(cmds, false)
}
