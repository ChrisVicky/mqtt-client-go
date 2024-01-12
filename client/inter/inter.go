package inter

type InterEnum int

type MqttInter interface {
	PubRegister()
	Pub(id InterEnum, b interface{})

	SubRegister()

	PrintRegister()
}
