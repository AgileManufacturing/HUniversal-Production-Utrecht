package nl.hu.client;

public interface ISubscriber{
	public void onMessage(String topic, Object message);
}
