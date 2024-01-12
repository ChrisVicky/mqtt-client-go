package main

import (
	"mqttclient/client"
	"mqttclient/config"
	"mqttclient/logger"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

func setMqttLog(c *config.Config) {
	mqtt.ERROR = logger.NewLog(c.Mqtt.Ef, c.ShowLog())
	mqtt.CRITICAL = logger.NewLog(c.Mqtt.Cf, c.ShowLog())
	mqtt.WARN = logger.NewLog(c.Mqtt.Wf, c.ShowLog())
	mqtt.DEBUG = logger.NewLog(c.Mqtt.Df, c.ShowLog())
}

func main() {
	// 1. Load Configuration
	conf := config.DefaultConfig
	if err := conf.LoadConfig(); err != nil {
		panic(err)
	}

	// 2. Setup Log Hookers ...
	logger.Setup(conf.GetLevel(), conf.Log.Fn)
	setMqttLog(conf)

	// 3. Setup Client
	r := client.NewClient(conf.GetMqttAddress(), conf.CConf, conf.CType)

	// 4. Connect to mqtt broker
	if token := r.Connect(); token.Wait() && token.Error() != nil {
		// Fatal Printout the Message & Exit Program with 1
		logger.Panicf("Connect to mqtt server error: %v", token.Error())
	}

	// 5. Inform server
	r.Online()

	// 6. Block & Served
	for r.Running() {
	}
	r.Offline()
	r.Disconnect(250)
	logger.Info("Exit Program")
	// TODO: 如果用户强行终止程序，是否应该发送 Offline
}
