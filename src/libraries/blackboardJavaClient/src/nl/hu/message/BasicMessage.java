package libraries.blackboardJavaClient.src.nl.hu.message;

public class BasicMessage{
	public String topic;
	public Object message;

	public BasicMessage(String topic, Object message){
		this.topic = topic;
		this.message = message;
	}
}
