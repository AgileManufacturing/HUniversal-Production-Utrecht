package generic;

public abstract class Mast {
	public enum State {
		OFFLINE, INITIALIZE, DEINITIALIZE,
		SAFE, SETUP, SHUTDOWN,
		STANDBY, START, STOP,
		NORMAL
	}
	public enum Mode {
		NORMAL, SERVICE, ERROR
	}
}
