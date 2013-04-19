package newDataClasses;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

public class DbData implements IMongoSaveable {
	private String ip;
	private int port;
	private String name;
	
	public DbData(String ip, int port, String name){
		this.ip = ip;
		this.port = port;
		this.name = name;
	}
	
	public DbData(BasicDBObject object){
		fromBasicDBObject(object);
	}

	/* (non-Javadoc)
	 * @see newDataClasses.DBSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start()
				.add("ip", ip)
				.add("port", port)
				.add("name", name).get();
	}

	/* (non-Javadoc)
	 * @see newDataClasses.DBSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		ip = object.getString("ip");
		port = object.getInt("port");
		name = object.getString("name");
	}

	/**
	 * @return the ip
	 */
	public String getIp() {
		return ip;
	}

	/**
	 * @param ip the ip to set
	 */
	public void setIp(String ip) {
		this.ip = ip;
	}

	/**
	 * @return the port
	 */
	public int getPort() {
		return port;
	}

	/**
	 * @param port the port to set
	 */
	public void setPort(int port) {
		this.port = port;
	}

	/**
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * @param name the name to set
	 */
	public void setName(String name) {
		this.name = name;
	}

	@Override
	public String toString() {
		return String.format("DbData [ip=%s, port=%s, name=%s]", ip, port, name);
	}
}
