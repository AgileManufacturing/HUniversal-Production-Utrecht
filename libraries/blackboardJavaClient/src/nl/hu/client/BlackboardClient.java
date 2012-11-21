package nl.hu.client;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import nl.hu.client.ISubscriber.BlackboardEvent;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;
import com.mongodb.Bytes;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.Mongo;
import com.mongodb.MongoInterruptedException;
import com.mongodb.util.JSON;

public class BlackboardClient extends Thread {

	private Mongo mongo;
	private HashMap<String, BasicDBObject> subscriptions = new HashMap<String, BasicDBObject>();
	private String collection;
	private String database;
	private ISubscriber callback;

	public BlackboardClient(String ip) {
		try {
			this.mongo = new Mongo(ip);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public BlackboardClient(String ip, int port) {
		try {
			this.mongo = new Mongo(ip, port);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void setDatabase(String database) {
		this.database = database;
	}

	public void setCollection(String collection) {
		this.collection = collection;
	}

	public void insert(Map map) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		System.out.println(map);
		mongo.getDB(database).getCollection(collection).insert(new BasicDBObject(map));
	}

	public void insertJson(String json) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		mongo.getDB(database).getCollection(collection).insert((DBObject)JSON.parse(json));
	}	
	
	public void remove(Map map) throws Exception
	{
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		mongo.getDB(database).getCollection(collection).remove(new BasicDBObject(map));
	}


	public void removeJson(String json) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		mongo.getDB(database).getCollection(collection).remove((DBObject)JSON.parse(json));
	}	
	
	public ArrayList<Map> get(Map query) throws Exception
	{
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}	
			BasicDBObject object = new BasicDBObject();
			object.put("$and", query);
			List<DBObject> found = mongo.getDB(database).getCollection(collection).find(object).toArray();
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
			throw new Exception("No collection selected");
		}
		ArrayList<String> jsons  = new ArrayList<String>();
		List<DBObject> found =  mongo.getDB(database).getCollection(collection).find((DBObject)JSON.parse(json)).toArray();
		for (DBObject obj: found) 
		{
				jsons.add(obj.toString());
		}	
		return jsons;
	}	
		
			
	public void update(Map query, Map set, Map unset) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		if(set == null){set= new HashMap();}
		if(unset == null){unset= new HashMap();}	
		BasicDBObject setObject = new BasicDBObject();
		setObject.put("$set", set);
		setObject.put("$unset", unset);
		System.out.println(new BasicDBObject(query));
		System.out.println(setObject);		
		mongo.getDB(database).getCollection(collection).findAndModify(new BasicDBObject(query), setObject);
	}


	public void updateJson(String query, String set, String unset) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		if(set == null){set="";}
		if(unset == null){unset="";}	
		BasicDBObject setObject = new BasicDBObject();
		setObject.put("$set", (DBObject)JSON.parse(set));
		setObject.put("$unset",(DBObject)JSON.parse(unset));
		System.out.println(query);
		System.out.println(setObject);		
		mongo.getDB(database).getCollection(collection).findAndModify((DBObject)JSON.parse(query), setObject);
	}
	


	public void subscribe(String topic) throws Exception {
		if (collection.isEmpty() || collection == null) {
			throw new Exception("No collection selected");
		} else if (database.isEmpty() || database == null) {
			throw new Exception("No collection selected");
		}
		subscriptions.put(topic, new BasicDBObject("topic", topic));
		if (subscriptions.size() == 1) {
			this.start();
		}
	}

	public void unsubscribe(String topic) {
		subscriptions.remove(topic);
		if (subscriptions.size() == 0) {
			this.interrupt();
		}
	}
	
	public void setCallBack(ISubscriber callback)
	{
		this.callback = callback;
	}

	public void run() {
		BasicDBObject where = new BasicDBObject();
		where.put("ns", database + "." + collection);

		DBCursor tailedCursor = mongo.getDB("local").getCollection("oplog.rs")
				.find(where).addOption(Bytes.QUERYOPTION_TAILABLE)
				.addOption(Bytes.QUERYOPTION_AWAITDATA);
		tailedCursor.skip(tailedCursor.size());
		String id = "";
		String operation = "";
		try {
			while (tailedCursor.hasNext()) {
				BasicDBObject addedObject = (BasicDBObject) tailedCursor.next();
				if (addedObject.containsField("o")) {
					id = ((BasicDBObject) addedObject.get("o"))
							.getString("_id");
				}
				if (addedObject.containsField("o2")
						&& (id == null || id.isEmpty())) {
					id = ((BasicDBObject) addedObject.get("o2"))
							.getString("_id");
				}
				operation = addedObject.getString("op");
				BasicDBObject messageCheckObject = new BasicDBObject();
				messageCheckObject.put("_id", new ObjectId(id));
				messageCheckObject.put("$or", subscriptions.values());
				BasicDBObject message = (BasicDBObject) mongo.getDB(database)
						.getCollection(collection).findOne(messageCheckObject);
				BlackboardEvent event = BlackboardEvent.UNKNOWN;
				if (message != null)
				{
					if(operation.equals("i"))
					{
						event = BlackboardEvent.ADD;
					}
					else if(operation.equals("u"))
					{
						event = BlackboardEvent.UPDATE;
					}
					else if(operation.equals("d"))
					{
						event = BlackboardEvent.REMOVE;
					}
					this.callback.onMessage(event, message.toMap());
					
				}
				else
				{
					if(operation.equals("d"))
					{
						event = BlackboardEvent.REMOVE;
						this.callback.onMessage(event, addedObject.toMap());
					}
				}
				
			}
		}

		catch (MongoInterruptedException exception) {
			System.out.println("Mongodriver interrupted because subscriptions count is zero");
		}

	}
}
