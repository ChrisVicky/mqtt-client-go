package cmd

type CMDEnum int

type CMDManager interface {
	StartCmd(CMDEnum) error
	StopCmd(CMDEnum) error
	CleanCmds()
}
