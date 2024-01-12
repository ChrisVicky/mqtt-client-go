# `Mqtt` Client

- Across Platform (`Golang`)
- Supports multiple client specifications
  - All they need is to implement in the `Client` interface
- Log systems

## TODO

- [ ] Add Other Clients
- [ ] `MQTT` Interfaces Specifications (Doc)

## File

```shell
.
├── client                  - Client Pkg
│   ├── client.go           - client Definition
│   ├── cmd                 - Cmd Definition (Interface)
│   │   └── cmd.go
│   └── robot               - Client Robot (Implementation) * Main Controller
│       ├── robot_cmd.go
│       ├── robot.go
│       └── robot.toml
├── config                  - Config Loader
│   ├── config.go
│   └── config.toml
├── config.toml
├── go.mod
├── go.sum
├── logger                  - Logger
│   └── log.go
├── main.go
├── Makefile
├── mosquitto               - mosquitto built (amd64)
│   ├── mosquitto.conf
│   └── mosquitto_linux
├── README.md
├── reply                   - mqtt reply helper
│   └── reply.go
├── robot.toml
└── runtime                 - runtime generated Logs & Stuff
    ├── app.log
    ├── mqtt_critical.log
    ├── mqtt_debug.log
    ├── mqtt_error.log
    └── mqtt_warn.log

9 directories, 24 files
```

## Usage

### 1. Build towards Platform Specification

- Example: Robot (`aarch64`)
- Building Options: `GOOS=linux GOARCH=arm64 go build .`

### 2. Copy configuration & binary application to Client

- `scp ./mqttclient config.toml robot.toml huanyu:~/Coding/mqtty_go/`

### 3. Check configuration on Robot

- [Check Configuration](#example-configuration)

### 4. Run Client

- `./mqttclient`

## Configuration

#### `config.toml`

- Example Configuration

```toml
# Mqtt Configuration
ctype = "ROBOT"                 # Client Type: ROBOT, WIFI, SPEAKER
cconfig = "./client/robot.toml" # Client Configuration File

[MQTT]
address = "127.0.0.1:1883"          # Mqtt Address
error_file = "mqtt_error.log"       # Error Log file
critical_file = "mqtt_critical.log" # Criti Log file
warn_file = "mqtt_warn.log"         # Warn Log file
debug_file = "mqtt_debug.log"       # Debug Log file
showlog = false                     # Show mqtt Log : false/true

[LOG]
level = "TRACE"      # "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL", "PANIC"
filename = "app.log" # Log file name
```

#### `robot.toml`

- Example Configuration

```toml
deviceId = "Robot1"
devType = "Robot"
```

## Extensions

- Up till now (01/12/2024), this application supports only Robot Client. Yet the architecture can boot other client specifications if they implement the `Client` and `CMDManager` interfaces.
