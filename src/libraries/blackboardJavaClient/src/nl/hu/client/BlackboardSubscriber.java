package nl.hu.client;

public interface BlackboardSubscriber{
	public void onMessage(MongoOperation operation, OplogEntry entry);
}
