package nl.hu.client;

import java.util.Map;

public interface ISubscriber {
	public enum BlackboardEvent
	{
		UNKNOWN,
		UPDATE,
		ADD,
		REMOVE
	}
	
	public void onMessage(BlackboardEvent event, String topic);
}
