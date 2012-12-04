package nl.hu.client;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;
import com.mongodb.Bytes;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.Mongo;
import com.mongodb.MongoInterruptedException;
import com.mongodb.util.JSON;

public class BlackboardClient extends Thread {

	private final String OPLOG = "oplog.rs";
	private final String LOCAL = "local";
	private final String OR_OPERAND = "$or";
	private final String AND_OPERAND = "$and";

	private DB OPLOG_DATABASE;
	private DBCollection OPLOG_COLLECTION;

	private Mongo mongo;
	private HashMap<String, BasicDBObject> subscriptions = new HashMap<String, BasicDBObject>();
	private String collection;
	private String database;
	private ISubscriber callback;
	private DB currentDatabase;
	private DBCollection currentCollection;



	public BlackboardClient(String ip) {
		try {
			this.mongo = new Mongo(ip);
			this.callback = callback;
			this.OPLOG_DATABASE = mongo.getDB(LOCAL);
			this.OPLOG_COLLECTION = OPLOG_DATABASE.getCollection(OPLOG);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public BlackboardClient(String ip, int port) {
		try {
			this.mongo = new Mongo(ip, port);
			this.callback = callback;
			this.OPLOG_DATABASE = mongo.getDB(LOCAL);
			this.OPLOG_COLLECTION = OPLOG_DATABASE.getCollection(OPLOG);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void setCallback(ISubscriber callback)
	{
		this.callback = callback;
	}

	public void setDatabase(String database)
	{
		this.database = database;
		currentDatabase = mongo.getDB(this.database);	
	}


	public void setCollection(String collection) throws Exception
	{
		if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}
		this.collection = collection;
		currentCollection = currentDatabase.getCollection(this.collection);
	}

	public void insertJson(String json) throws Exception {
		if(collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}
		currentCollection.insert((DBObject)JSON.parse(json));
	}	
	
	public void removeJson(String json) throws Exception {
		if(collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}
		currentCollection.remove((DBObject)JSON.parse(json));
	}	
	
	public ArrayList<Map> get(Map query) throws Exception {
		if(collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}	
			BasicDBObject object = new BasicDBObject();
			object.put(AND_OPERAND, query);
			List<DBObject> found = currentCollection.find(object).toArray();
			ArrayList<Map> maps  = new ArrayList<Map>();
			
			for (DBObject obj: found) 
			{
				maps.add(obj.toMap());
			}	
			return maps;
	}


	public ArrayList<String> getJson(String json) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}
		ArrayList<String> jsons = new ArrayList<String>();
		List<DBObject> found = currentCollection.find((DBObject)JSON.parse(json)).toArray();
		for (DBObject obj : found) 
		{
				jsons.add(obj.toString());
		}	
		return jsons;
	}	

	
	public void read(boolean blocked) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		} else if(subscriptions.size() == 0) {
			throw new Exception("No subscribtions has been found");
		}

		BasicDBObject messageCheckObject = new BasicDBObject();
		messageCheckObject.put(OR_OPERAND, subscriptions.values());
		BasicDBObject message = (BasicDBObject) currentCollection.findOne(messageCheckObject);
		if(message!= null) {
			callback.onMessage(message.toString());
		} else if(blocked && subscriptions.size() > 0) {
			BasicDBObject where = new BasicDBObject();
			where.put("ns", database + "." + collection);
			
			DBCursor tailedCursor = OPLOG_COLLECTION.find().addOption(Bytes.QUERYOPTION_TAILABLE)
					.addOption(Bytes.QUERYOPTION_AWAITDATA);

			tailedCursor.skip(tailedCursor.size());
			tailedCursor.hasNext();
			message = (BasicDBObject) currentCollection.findOne(messageCheckObject);
			
			if(message!= null) {
				callback.onMessage(message.toString());
			}
		}
	}		

	public void removeFirst() {
		BasicDBObject messageCheckObject = new BasicDBObject();
		BasicDBObject message = (BasicDBObject) currentCollection.findOne();
		currentCollection.remove(message);
	}

	public void updateJson(String query, String set, String unset) throws Exception {
		if(collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}
		if(set == null) { set = ""; }
		if(unset == null) { unset = ""; }	
		BasicDBObject setObject = new BasicDBObject();
		setObject.put("$set", (DBObject)JSON.parse(set));
		setObject.put("$unset",(DBObject)JSON.parse(unset));
		System.out.println(query);
		System.out.println(setObject);		
		currentCollection.findAndModify((DBObject)JSON.parse(query), setObject);
	}
	
	public void subscribe(String topic) throws Exception {
		if(collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No database selected");
		}
		subscriptions.put(topic, new BasicDBObject("topic", topic));
	}

	public void unsubscribe(String topic) {
		subscriptions.remove(topic);
	}
}
