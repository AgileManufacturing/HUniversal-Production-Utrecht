package rexos.utillities.log;

public enum LogLevel {

	EMERGENCY(7),
	ALERT(6),
	CRITICAL(5),
	ERROR(4),
	WARNING(3),
	NOTIFICATION(2),
	INFORMATION(1),
	DEBUG(0);

	private int level;
	
	private LogLevel(int level)
	{
		this.level = level;
	}
	
	public int getLevel()
	{
		return level;
	}

}
