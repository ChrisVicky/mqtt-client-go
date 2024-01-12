package robot

import (
	"fmt"
	"mqttclient/client/cmd"
	"mqttclient/client/inter"
	"mqttclient/logger"
	"mqttclient/reply"
	"os"
	"os/exec"
	"sync"

	"github.com/BurntSushi/toml"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

const (
	mapName           = "map"
	defaultConfigName = "robot.toml"
)

const (
	online inter.InterEnum = iota
	offline
	pubStart
	logInfo
	logError
	sendPgm
)

type Robot struct {
	mqtt.Client
	cmd.CMDManager
	inter.MqttInter

	Config     string // Configuration File
	DeviceId   string `toml:"deviceId"`
	DeviceType string `toml:"devType"`

	stat string // on / off

	// To Implement inter.MqttInter
	pubInterfaces map[inter.InterEnum]string
	subInterfaces []string
}

func NewRobot(opt *mqtt.ClientOptions, fn string) *Robot {
	r := &Robot{
		Client: mqtt.NewClient(opt),
		Config: fn,

		CMDManager: &rcm{
			cmdRecords: make(map[cmd.CMDEnum]*exec.Cmd),
			cmdsMu:     &sync.Mutex{},
		},

		pubInterfaces: make(map[inter.InterEnum]string),
		subInterfaces: []string{},
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

// Pub Register -- Register Interfaces
func (r *Robot) PubRegister() {
	r.pubInterfaces = map[inter.InterEnum]string{
		online:  "/client/online",
		offline: "/client/offline",

		pubStart: r.baseApi() + "/start",
		logInfo:  r.baseApi() + "/info",
		logError: r.baseApi() + "/error",
		sendPgm:  r.baseApi() + "/map/pgm",
	}
}

func (r *Robot) Pub(id inter.InterEnum, b interface{}) {
	if add, ok := r.pubInterfaces[id]; ok {
		tk := r.Publish(add, 1, false, b)
		tk.Wait()
		logger.Tracef("%v Published", add)
	} else {
		logger.Errorf("Unkown Interface id: %v", id)
	}
}

func (r *Robot) PrintRegister() {
	for _, v := range r.pubInterfaces {
		logger.Infof("[Pub] %v", v)
	}
	for _, v := range r.subInterfaces {
		logger.Infof("[Sub] %v", v)
	}
}

// Online (Called by client end)
// 1. Load Config
// 2. Register Sub & Pub Interfaces
// 3. Change stat & Inform Server
func (r *Robot) Online() {
	if err := r.loadConfig(); err != nil {
		logger.Fatal(err)
	}

	logger.Info("Config Loaded")

	r.SubRegister()
	r.PubRegister()
	r.PrintRegister()

	r.stat = "on"
	r.Pub(online, reply.Ok(r.parseInform()))
}

func (r *Robot) Offline() {
	r.stat = "off"
	r.Pub(offline, "")
}

func (r *Robot) pubStart() {
	r.Pub(pubStart, "")
}

func (r *Robot) logError(str string) {
	r.Pub(logError, str)
}

func (r *Robot) logInfo(str string) {
	r.Pub(logInfo, str)
}

func (r *Robot) sendPgm(payload []byte) {
	r.Pub(sendPgm, payload)
}

// Register Subscription
func (r *Robot) SubRegister() {
	subscriptions := map[string]func(mqtt.Client, mqtt.Message){
		// [sub] Init Status
		r.baseApi() + "/map/config": func(mqtt.Client, mqtt.Message) {
			go r.init()
		},

		// [sub] start Map Generation
		r.baseApi() + "/map/start": func(mqtt.Client, mqtt.Message) {
			go r.pubStart()
			go r.mapGeneration()
		},

		// [sub] Send Pgm through /map/pgm
		r.baseApi() + "/map/fetch": func(mqtt.Client, mqtt.Message) {
			go r.pubStart()
			go func() {
				r.mapBuild()
				// Read map from disk & sent via publish
				if f, err := os.ReadFile(mapName + ".pgm"); err != nil {
					// Error Emit
					r.logError(fmt.Sprintf("Wrong: %v", err))
					logger.Infof("Error Reading File: %v, %v", mapName+".pgm", err)
				} else {
					// This Do not need to wait
					logger.Infof("Sending %v to %v", mapName+".pgm", r.baseApi()+"/map/pgm")
					r.sendPgm(f)
				}
			}()
		},

		// [sub] Refresh Cmds (Same as Init)
		r.baseApi() + "/refresh": func(mqtt.Client, mqtt.Message) {
			go r.init()
		},

		// [sub] Exit Program
		r.baseApi() + "/bye": func(mqtt.Client, mqtt.Message) {
			logger.Infof("Receive Bye, Start Cleaning Cmds")
			go r.CleanCmds()
			r.stat = "off"
		},

		// [sub] /test
		r.baseApi() + "/test": func(mqtt.Client, mqtt.Message) {
			logger.Infof("Testing Called")
		},
	}

	for k, v := range subscriptions {
		tk := r.Subscribe(k, 1, v)
		tk.Wait()
		r.subInterfaces = append(r.subInterfaces, k)
	}
}

func (r *Robot) init() {
	logger.Infof("Start Init")
	r.CleanCmds()
	r.RunCmdAsync(MAIN_NODE)
	r.RunCmdAsync(PS2)
}

func (r *Robot) mapGeneration() {
	r.RunCmdAsync(G_MAPPING)
}

func (r *Robot) mapBuild() {
	r.RunCmd(BUILD_MAP)
}
