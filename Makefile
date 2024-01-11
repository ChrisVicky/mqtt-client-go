build:
	GOOS=linux GOARCH=arm64 go build .

publish:
	scp ./mqttclient huanyu:~/Coding/mqtt_go/
