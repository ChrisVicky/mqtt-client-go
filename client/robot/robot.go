package robot

import (
	"fmt"
	"mqttclient/client/cmd"
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

	cmd.CMDManager

	Config     string // Configuration File
	DeviceId   string `toml:"deviceId"`
	DeviceType string `toml:"devType"`

	// Stat
	stat   string // on / off
	params string
}

func NewRobot(opt *mqtt.ClientOptions, fn string) *Robot {
	r := &Robot{
		Client: mqtt.NewClient(opt),
		Config: fn,
		CMDManager: &rcm{
			cmdRecords: make(map[cmd.CMDEnum]*exec.Cmd),
			cmdsMu:     &sync.Mutex{},
		},
	}
	return r
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

// Online (Called by client end)
// 1. Load Config
// 2. Register Subscription
// 3. Change stat & Inform Server
func (r *Robot) Online() {
	if err := r.loadConfig(); err != nil {
		logger.Fatal(err)
	}

	logger.Info("Config Loaded")

	r.subRegistration()

	r.stat = "on"
	tk := r.Publish("/client/online", 1, false, reply.Ok(r.parseInform()))
	tk.Wait()
}

// Offline (Triggered by both ends)
// Change stat & Inform Server
func (r *Robot) Offline() {
	r.stat = "off"
	tk := r.Publish("/client/offline", 1, false, reply.Ok(r.parseInform()))
	tk.Wait()
}

// Shall get called when work starts
func (r *Robot) pubStart() {
	tk := r.Publish(r.baseApi()+"/start", 1, false, "")
	tk.Wait()
}

func (r *Robot) loginfo(str string) {
	tk := r.Publish(r.baseApi()+"/info", 0, false, str)
	tk.Wait()
}

func (r *Robot) logerror(str string) {
	tk := r.Publish(r.baseApi()+"/error", 0, false, str)
	tk.Wait()
}

// Register Subscription
func (r *Robot) subRegistration() {
	subscriptions := map[string]func(mqtt.Client, mqtt.Message){
		// [sub] base/map/start
		r.baseApi() + "/map/start": func(mqtt.Client, mqtt.Message) {
			r.pubStart()
			r.mapGeneration()
		},

		// [sub] base/map/fetch
		r.baseApi() + "/map/fetch": func(mqtt.Client, mqtt.Message) {
			r.pubStart()

			r.mapBuild()

			// Read map from disk & sent via publish
			if f, err := os.ReadFile(mapName + ".pgm"); err != nil {
				// Error Emit
				r.logerror(fmt.Sprintf("Wrong: %v", err))
				logger.Infof("Error Reading File: %v, %v", mapName+".pgm", err)
			} else {
				// This Do not need to wait
				logger.Infof("Sending %v to %v", mapName+".pgm", r.baseApi()+"/map/pgm")
				r.Publish(r.baseApi()+"/map/pgm", 0, false, f)
			}
		},

		// [sub] /refresh
		r.baseApi() + "/refresh": func(mqtt.Client, mqtt.Message) {
			r.CleanCmds()
		},

		// [sub] /bye
		r.baseApi() + "/bye": func(mqtt.Client, mqtt.Message) {
			logger.Warn("Receive Bye, Start Cleaning Cmds")
			r.CleanCmds()
			r.stat = "off"
		},

		// [sub] /test
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

func (r *Robot) init() {
}

func (r *Robot) mapGeneration() {
	r.RunCmdAsync(MAIN_NODE)
	r.RunCmdAsync(PS2)
	time.Sleep(2 * time.Second)

	r.RunCmdAsync(G_MAPPING)
}

func (r *Robot) mapBuild() {
	r.RunCmd(BUILD_MAP)
}
