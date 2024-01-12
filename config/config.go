package config

import (
	"fmt"
	"os"

	"github.com/BurntSushi/toml"
	"github.com/sirupsen/logrus"
)

type Config struct {
	FileName string
	CConf    string `toml:"cconfig"`
	CType    string `toml:"ctype"`

	Mqtt MQTT
	Log  LOG
}

type MQTT struct {
	// Mqtt Adress
	Address string `toml:"address"`

	// Mqtt Log
	Ef string `toml:"error_file"`
	Cf string `toml:"critical_file"`
	Wf string `toml:"warn_file"`
	Df string `toml:"debug_file"`
	Sl bool   `toml:"showlog"`
}

func (m *MQTT) clean() {
	fmt.Printf("Mqtt Configuration:\t%+v\n", m)
}

func (c *Config) GetMqttAddress() string {
	return c.Mqtt.Address
}

func (c *Config) ShowLog() bool {
	return c.Mqtt.Sl
}

type LOG struct {
	Level string `toml:"level"`
	Fn    string `toml:"filename"`
	level logrus.Level
}

func (l *LOG) clean() {
	lm := map[string]logrus.Level{
		"TRACE": logrus.TraceLevel,
		"DEBUG": logrus.DebugLevel,
		"INFO":  logrus.InfoLevel,
		"WARN":  logrus.WarnLevel,
		"ERROR": logrus.ErrorLevel,
		"FATAL": logrus.FatalLevel,
		"PANIC": logrus.PanicLevel,
	}
	l.level = lm[l.Level]
	fmt.Printf("Log Configuration:\t%+v\n", l)
}

func (c *Config) GetLevel() logrus.Level {
	return c.Log.level
}

var (
	DefaultFile   = "config.toml"
	DefaultConfig = &Config{FileName: DefaultFile}
)

func NewConfig() *Config {
	return &Config{FileName: DefaultFile}
}

func (c *Config) LoadConfig() (err error) {
	var cBytes []byte

	fn := c.FileName

	if _, err = os.Stat(fn); err != nil {
		fn = DefaultFile
	}

	if cBytes, err = os.ReadFile(fn); err != nil {
		return
	}

	if err = toml.Unmarshal(cBytes, c); err != nil {
		return
	}

	return c.clean()
}

func (c *Config) clean() error {
	c.Mqtt.clean()
	c.Log.clean()

	switch c.CType {
	case "ROBOT":
		return nil
	default:
		return fmt.Errorf("unknown type: %v", c.CType)
	}
}

func ReadConfigMap(fn string) (m map[string]interface{}, err error) {
	var cBytes []byte
	if _, err = os.Stat(fn); err != nil {
		fn = DefaultFile
	}
	if cBytes, err = os.ReadFile(fn); err != nil {
		return
	}

	if err = toml.Unmarshal(cBytes, &m); err != nil {
		return
	}
	return
}
