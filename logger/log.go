package logger

import (
	"mqttclient/config"
	"os"
	"path/filepath"

	"github.com/rifflock/lfshook"
	"github.com/sirupsen/logrus"
	prefixed "github.com/x-cray/logrus-prefixed-formatter"
)

var (
	logger  *logrus.Logger
	RUNTIME = "./runtime"
	conf    = config.DefaultConfig
)

// CustomFormatter 自定义日志格式化器
type CustomFormatter struct{}

// NullWriter 是一个不执行任何操作的 io.Writer。
type NullWriter struct{}

// Write 实现了 io.Writer 接口。
func (*NullWriter) Write(p []byte) (n int, err error) {
	return len(p), nil
}

func Setup(level logrus.Level, fn string) {
	// Mkdir & Return error if it exists
	os.Mkdir(RUNTIME, os.ModePerm)

	logger = logrus.New()

	logger.SetLevel(level)
	logger.SetFormatter(
		&prefixed.TextFormatter{
			DisableColors:   false,
			TimestampFormat: "2006-01-02 15:04:05",
			FullTimestamp:   true,
			ForceFormatting: true,
		},
	)

	// Define hooker writes to local file
	logFile, err := os.OpenFile(filepath.Join(RUNTIME, fn), os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0666)
	if err != nil {
		logger.Fatal("Failed to log to file, using default stderr")
	}

	fileHook := lfshook.NewHook(lfshook.WriterMap{
		logrus.InfoLevel:  logFile,
		logrus.WarnLevel:  logFile,
		logrus.ErrorLevel: logFile,
		logrus.FatalLevel: logFile,
		logrus.PanicLevel: logFile,
	}, &logrus.JSONFormatter{})

	logger.AddHook(fileHook)
}

func Trace(args ...interface{}) {
	logger.Trace(args...)
}

func Tracef(str string, args ...interface{}) {
	logger.Tracef(str, args...)
}

func Debug(args ...interface{}) {
	logger.Debug(args...)
}

func Debugf(str string, args ...interface{}) {
	logger.Debugf(str, args...)
}

func Info(args ...interface{}) {
	logger.Info(args...)
}

func Infof(str string, args ...interface{}) {
	logger.Infof(str, args...)
}

func Print(args ...interface{}) {
	logger.Print(args...)
}

func Printf(str string, args ...interface{}) {
	logger.Printf(str, args...)
}

func Warn(args ...interface{}) {
	logger.Warn(args...)
}

func Warnf(str string, args ...interface{}) {
	logger.Warnf(str, args...)
}

func Error(args ...interface{}) {
	logger.Error(args...)
}

func Errorf(str string, args ...interface{}) {
	logger.Errorf(str, args...)
}

func Fatal(args ...interface{}) {
	logger.Fatal(args...)
}

func Fatalf(str string, args ...interface{}) {
	logger.Fatalf(str, args...)
}

func Panic(args ...interface{}) {
	logger.Panic(args...)
}

func Panicf(str string, args ...interface{}) {
	logger.Panicf(str, args...)
}

func NewLog(fileName string, out bool) *logrus.Logger {
	ret := logrus.New()

	if out {
		ret.SetOutput(os.Stdout)
		ret.SetLevel(logrus.TraceLevel)
		ret.SetFormatter(
			&prefixed.TextFormatter{
				DisableColors:   false,
				TimestampFormat: "2006-01-02 15:04:05",
				FullTimestamp:   true,
				ForceFormatting: true,
			},
		)
	} else {
		ret.SetOutput(&NullWriter{})
	}

	if fileName != "" {
		logFile, err := os.OpenFile(filepath.Join(RUNTIME, fileName), os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0666)
		if err != nil {
			logger.Fatal("Failed to log to file, using default stderr")
		}

		// 创建文件写入器，设置 JSONFormatter
		fileHook := lfshook.NewHook(lfshook.WriterMap{
			logrus.InfoLevel:  logFile,
			logrus.WarnLevel:  logFile,
			logrus.ErrorLevel: logFile,
			logrus.FatalLevel: logFile,
			logrus.PanicLevel: logFile,
		}, &logrus.JSONFormatter{})

		ret.AddHook(fileHook)
	}

	return ret
}
