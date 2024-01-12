package cmd

type CMDEnum int

type CMDManager interface {
	RunCmdAsync(CMDEnum) error
	RunCmd(CMDEnum) error
	StopCmd(CMDEnum) error
	CleanCmds()
}
