package nl.hu.client;

public enum MongoOperation {
	INSERT("i"),
	UPDATE("u"),
	NOOP("n"),
	DELETE("d");
	
	private final String opCode;

	private MongoOperation(String opCode) {
		this.opCode = opCode;
	}

	public String getOpCode() {
		return opCode;
	}
	
	public static MongoOperation get(String opCode) {
		for (MongoOperation op : values()) {
			if (op.opCode.equals(opCode)) {
				return op;
			}
		}
		
		return null;
	}
}
