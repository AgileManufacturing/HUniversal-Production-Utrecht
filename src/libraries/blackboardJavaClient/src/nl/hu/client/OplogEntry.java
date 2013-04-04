package nl.hu.client;

import com.mongodb.DBObject;

public class OplogEntry {
	private static final String NAMESPACE_FIELD = "ns";
	private static final String TIMESTAMP_FIELD = "ts";
	private static final String OPERATION_FIELD = "op";
	private static final String UID_FIELD = "h";
	private static final String UPDATE_DOC_FIELD = "o";
	private static final String UPDATE_CRITERIA_FIELD = "o2";
	
	private DBObject oplogEntry;

	public OplogEntry(DBObject oplogEntry) {
		this.oplogEntry = oplogEntry;
	}
	
	public MongoOperation getOperation() {
		Object obj = oplogEntry.get(OPERATION_FIELD);
		return MongoOperation.get((String)obj);
	}
	
	public String getNamespace() {
		return oplogEntry.get(NAMESPACE_FIELD).toString();
	}
	
	public String getUpdateDocument() { 
		return oplogEntry.get(UPDATE_DOC_FIELD).toString();
	}
	
	public String getUpdateCriteria() {
		return oplogEntry.get(UPDATE_CRITERIA_FIELD).toString();
	}
	
	public String toString() {
		return oplogEntry.toString();
	}
}
