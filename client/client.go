package client

import (
	"mqttclient/logger"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

const (
	ROBOT = iota
)

type MyClient interface {
	mqtt.Client

	Online()
	Offline()
	Running() bool

	subRegistration()
	loadConfig() error
	baseApi() string

	// TODO:
	// ReturnLog()
}

// TODO: Add Choices of Mode
// NOTE: New a client
func NewClient(address string, config string, t int) MyClient {
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
	case ROBOT:
		r = &Robot{Client: mqtt.NewClient(opt), Config: config}
	default:
		logger.Fatalf("Type %v NOT implemented", t)
	}

	return r
}
