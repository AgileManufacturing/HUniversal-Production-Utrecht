package util.log;

public enum LogLevel {

	EMERGENCY(7, "EMERG"),
	ALERT(6, "ALERT"),
	CRITICAL(5, "CRIT"),
	ERROR(4, "ERROR"),
	WARNING(3, "WARN"),
	NOTIFICATION(2, "NOTE"),
	INFORMATION(1, "INFO"),
	DEBUG(0, "DEBUG");

	private int level;
	private String name;
	
	private LogLevel(int level, String name) {
		this.level = level;
		this.name = name;
	}
	
	public int getLevel() {
		return level;
	}
	public String getName() {
		return name;
	}

}
