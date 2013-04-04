package nl.hu.client;


public class BlackboardSubscription {
	private MongoOperation operation;
	private BlackboardSubscriber subscriber;
	
	public BlackboardSubscription(MongoOperation operation, BlackboardSubscriber subscriber) {
		this.operation = operation;
		this.subscriber = subscriber;
	}

	public MongoOperation getOperation() {
		return operation;
	}

	public BlackboardSubscriber getSubscriber() {
		return subscriber;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((operation == null) ? 0 : operation.hashCode());
		result = prime * result
				+ ((subscriber == null) ? 0 : subscriber.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		BlackboardSubscription other = (BlackboardSubscription) obj;
		if (operation != other.operation)
			return false;
		if (subscriber == null) {
			if (other.subscriber != null)
				return false;
		} else if (!subscriber.equals(other.subscriber))
			return false;
		return true;
	}
}
