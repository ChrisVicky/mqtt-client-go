package reply

import (
	"encoding/json"
	"time"
)

type Reply struct {
	Data      interface{} `json:"data"`
	Timestamp string      `json:"timestamp"`
}

func Ok(d interface{}) (b []byte) {
	r := Reply{
		Timestamp: time.Now().String(),
		Data:      d,
	}
	b, _ = json.Marshal(r)
	return
}
