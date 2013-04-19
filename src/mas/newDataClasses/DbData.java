package newDataClasses;

import java.io.Serializable;

import com.mongodb.BasicDBObject;

public class DbData implements Serializable{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	public String ip;
	public int port;
	public String name;
	
	public DbData(String ip, int port, String name){
		this.ip = ip;
		this.port = port;
		this.name = name;
	}
	
	public DbData() {}

	public void fromBasicDBObject(BasicDBObject dbData){
		this.ip = dbData.getString("ip");
		this.port = dbData.getInt("port");
		this.name = dbData.getString("name");
	}
	
	public BasicDBObject toBasicDBObject(){
		BasicDBObject dbData = new BasicDBObject("ip", ip);
		dbData.put("port", port);
		dbData.put("name", name);
		return dbData;
	}
}
