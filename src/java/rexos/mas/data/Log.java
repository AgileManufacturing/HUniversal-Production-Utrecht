package rexos.mas.data;

import com.mongodb.BasicDBObject;

public class Log{
	/**
	 * @var log
	 * 		The log.
	 */
	private BasicDBObject log;

	/**
	 * Empty constructor for the Log.
	 */
	public Log(){
		log = new BasicDBObject();
	}
	
	/**
	 * Constructor for initializing the log with the new log variable
	 * @param log The new log.
	 */
	public Log(BasicDBObject log){
		this.log = log;
	}
	
	/**
	 * method for adding a new entry to the log.
	 * @param key The key of the entry.
	 * @param value The value of the entry
	 */
	public void log(String key, BasicDBObject value){
		log.put(key, value);
	}
	
	/**
	 * Getter for the log
	 * @return the log
	 */
	public BasicDBObject getLog(){
		return log;
	}
}
