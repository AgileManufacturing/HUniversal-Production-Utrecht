package rexos.mas.newDataClasses;

import com.mongodb.BasicDBObject;

public class DbData implements IMongoSaveable {
	public String ip;
	public int port;
	public String name;
	
	public DbData(String ip, int port, String name){
		this.ip = ip;
		this.port = port;
		this.name = name;
	}
	
	public DbData(BasicDBObject dbData){
		fromBasicDBObject(dbData);
	}

	@Override
	public void fromBasicDBObject(BasicDBObject dbData){
		this.ip = dbData.getString("ip");
		this.port = dbData.getInt("port");
		this.name = dbData.getString("name");
	}
	
	@Override
	public BasicDBObject toBasicDBObject(){
		BasicDBObject dbData = new BasicDBObject("ip", ip);
		dbData.put("port", port);
		dbData.put("name", name);
		return dbData;
	}
}
