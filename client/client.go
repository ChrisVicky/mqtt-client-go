package client

import (
	"mqttclient/client/cmd"
	"mqttclient/client/robot"
	"mqttclient/logger"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type MyClient interface {
	mqtt.Client

	Online()
	Offline()
	Running() bool

	cmd.CMDManager
}

// New a client
// address: ip address
// config: config path
// t: client type
func NewClient(address string, config string, t string) MyClient {
	opt := mqtt.NewClientOptions()
	opt.AddBroker(address)
	opt.OnConnect = func(c mqtt.Client) {
		logger.Infof("Connected! %v", c.IsConnected())
	}

	opt.OnConnectionLost = func(c mqtt.Client, err error) {
		logger.Infof("Connection Lost: %v %v", err, c.IsConnectionOpen())
	}
	var r MyClient

	switch t {
	case "ROBOT":
		r = robot.NewRobot(opt, config)
	default:
		logger.Fatalf("Type %v NOT implemented", t)
	}

	return r
}
