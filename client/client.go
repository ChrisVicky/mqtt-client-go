package client

import (
	"mqttclient/client/cmd"
	"mqttclient/client/inter"
	"mqttclient/client/robot"
	"mqttclient/logger"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type MyClient interface {
	mqtt.Client // Mqtt has Implemented

	cmd.CMDManager  // Customized Cmd Manager
	inter.MqttInter // CUstomized Mqtt Interfaces Manager

	Online()
	Offline()
	Running() bool
}

// New a client
// address: broker ip address
// config: config path
// t: client type
func NewClient(address string, config string, t string) MyClient {
	opt := mqtt.NewClientOptions()
	opt.AddBroker(address)
	opt.OnConnect = func(mqtt.Client) {
		logger.Infof("Connected to %v", address)
	}

	opt.OnConnectionLost = func(c mqtt.Client, err error) {
		logger.Infof("Connection Lost: %v %v", err, c.IsConnectionOpen())
	}
	var r MyClient

	// TODO: Add New Client Choice HERE
	switch t {
	case "ROBOT":
		r = robot.NewRobot(opt, config)
	default:
		logger.Fatalf("Type %v NOT implemented", t)
	}

	return r
}
