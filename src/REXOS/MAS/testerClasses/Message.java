package MAS.testerClasses;

public class Message{
	private String name;
	private int id;
	private long timeSend;
	private long timeReceived;
	
	public Message(String agentName, int messageID, long timeSend, long timeReceived) {
		name = agentName;
		id = messageID;
		this.timeSend = timeSend;
		this.timeReceived = timeReceived;
	}
	
	public String getName(){
		return name;
	}
	public int getID(){
		return id;
	}
	public long getTimeSend(){
		return timeSend;
	}
	public long getTimeReceived(){
		return timeReceived;
	}

}