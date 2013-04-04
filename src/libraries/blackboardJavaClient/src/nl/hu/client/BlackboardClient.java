/**
 * @file BlackboardClient.java
 * @brief Class representing a blackboard connection.
 * @date Created: 2012-04-04
 *
 * @author 1.0 Dick van der Steen
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * @note 2013-04-04 JWW: Generalized BlackboardClient. Can now be used for more than just listening to topics.
 **/

package nl.hu.client;

import com.mongodb.*;
import com.mongodb.util.JSON;

import java.util.ArrayList;
import java.util.List;

/**
 * Client class for a mongodb blackboard.
 **/
public class BlackboardClient {
	/**
	 * @var String OPLOG
	 * Operation log collection name of MongoDB.
	 **/
	private final String OPLOG = "oplog.rs";

	/**
	 * @var String LOCAL
	 * Local database name of MongoDB.
	 **/
	private final String LOCAL = "local";

	/**
	 * @var Mongo mongo
	 * Connection object to MongoDB.
	 **/
	private Mongo mongo;

	/**
	 * @var HashMap<String, BasicDBObject> subscriptions
	 * Link between subscribed topic name and MongoDbs BasicDBObjects
	 **/
	private ArrayList<BlackboardSubscription> subscriptions;

	/**
	 * @var String collection
	 * Name of the used MongoDB collection
	 **/
	private String collection;

	/**
	 * @var String database
	 * Name of the used MongoDB database
	 **/
	private String database;

	/**
	 * @var DB currentDatabase
	 * Database object of the currently used database
	 **/
	private DB currentDatabase;

	/**
	 * @var DBCollection currentCollection
	 * DBCollection object of the currently used collection
	 **/
	private DBCollection currentCollection;

	/**
	 * @var DBCursor tailedCursor
	 * TailedCursor for tracking changes on the operation log of MongoDB
	 **/
	private DBCursor tailedCursor;

	/**
	 * @var DB oplogDatabase
	 * Database object of the oplog database
	 **/
	private DB oplogDatabase;

	/**
	 * @var DBCollection oplogCollection
	 * DBCollection object of the oplog collection
	 **/
	private DBCollection oplogCollection;

	private TailedCursorThread tcThread;
	
	/**
	 * Constructor of BlackboardClient.
	 *
	 * @param host The mongoDB host.
	 * @param subscriber The subscriber.
	 **/
	public BlackboardClient(String host) {
		try {
			this.subscriptions = new ArrayList<BlackboardSubscription>();
			this.mongo = new Mongo(host);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Constructor of BlackboardClient.
	 *
	 * @param host The ip of the MongoDB host.
	 * @param port The port of the MongoDB host.
	 * @param subscriber The subscriber.
	 **/
	public BlackboardClient(String host, int port) {
		try {
			this.subscriptions = new ArrayList<BlackboardSubscription>();
			this.mongo = new Mongo(host, port);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Sets the database to use.
	 *
	 * @param database The database to load from MongoDB.
	 * @throws Exception 
	 **/
	public void setDatabase(String database) throws Exception {
		if (database == null || database.isEmpty()) {
			throw new Exception("Database name cannot be empty.");
		}
		this.database = database;
		currentDatabase = mongo.getDB(this.database);
	}

	/**
	 * Sets the collection to use.
	 *
	 * @param collection The collection to load from MongoDB.
	 **/
	public void setCollection(String collection) throws Exception {
		if (currentDatabase == null) {
			throw new Exception("No database selected");
		}
		this.collection = collection;
		currentCollection = currentDatabase.getCollection(this.collection);
	}

	/**
	 * Inserts document into MongoDB using json format. Will throw an exception if collection or database has not been set.
	 *
	 * @param json The json format for the MongoDB insert statement.
	 **/
	public void insertJson(String json) throws Exception {
		if (collection == null || collection.isEmpty()) {
			throw new Exception("No collection selected");
		} else if (database == null || database.isEmpty()) {
			throw new Exception("No database selected");
		}
		currentCollection.insert((DBObject) JSON.parse(json));
	}

	/**
	 * Removes document from MongoDB using json format. Will throw an exception if collection or database has not been set.
	 *
	 * @param json The json format for the MongoDB remove statement.
	 **/
	public void removeJson(String json) throws Exception {
		if (collection == null || collection.isEmpty()) {
			throw new Exception("No collection selected");
		} else if (database == null || database.isEmpty()) {
			throw new Exception("No database selected");
		}
		currentCollection.remove((DBObject) JSON.parse(json));
	}

	/**
	 * Queries MongoDB using json format, will throw an exception if collection or database has not been set.
	 *
	 * @param json The json format for the MongoDB query statement.
	 **/
	public ArrayList<String> getJson(String json) throws Exception {
		if (collection == null || collection.isEmpty()) {
			throw new Exception("No collection selected");
		} else if (database == null || database.isEmpty()) {
			throw new Exception("No database selected");
		}
		ArrayList<String> jsons = new ArrayList<String>();
		List<DBObject> found = currentCollection.find((DBObject) JSON.parse(json)).toArray();
		for (DBObject obj : found) {
			jsons.add(obj.toString());
		}
		return jsons;
	}

	/**
	 * Removes first message on blackboard
	 **/
	public void removeFirst() {
		BasicDBObject message = (BasicDBObject) currentCollection.findOne();
		currentCollection.remove(message);
	}

	/**
	 * Updates document from MongoDB using json format. Will throw an exception if collection or database has not been set.
	 *
	 * @param query The json format for the MongoDB query statement.
	 * @param set The json format for the MongoDB set statement.
	 * @param unset The json format for the MongoDB unset statement.
	 **/
	public void updateJson(String query, String set, String unset) throws Exception {
		if (collection == null || collection.isEmpty()) {
			throw new Exception("No collection selected");
		} else if (database == null || database.isEmpty()) {
			throw new Exception("No database selected");
		}
		if (set == null) {
			set = "";
		}
		if (unset == null) {
			unset = "";
		}
		BasicDBObject setObject = new BasicDBObject();
		setObject.put("$set", (DBObject) JSON.parse(set));
		setObject.put("$unset", (DBObject) JSON.parse(unset));
		System.out.println(query);
		System.out.println(setObject);
		currentCollection.findAndModify((DBObject) JSON.parse(query), setObject);
	}
	

	public void subscribe(BlackboardSubscription sub) throws Exception {
		if (collection == null || collection.isEmpty()) {
			throw new Exception("No collection selected");
		} else if (database == null || database.isEmpty()) {
			throw new Exception("No database selected");
		}
		subscriptions.add(sub);
		if (tcThread == null) {
			tcThread = new TailedCursorThread();
			tcThread.start();
		}
	}


	public void unsubscribe(BlackboardSubscription sub) {
		subscriptions.remove(sub);
		if (subscriptions.size() == 0) {
			tcThread.interrupt();
			tcThread = null;
		}
	}
	
	/**
	 * Class for tailable cursor thread within the client
	 **/
	public class TailedCursorThread extends Thread {
		/**
		 * Constructor of TailedCursorThread.
		 **/
		public TailedCursorThread() {
			oplogDatabase = mongo.getDB(LOCAL);
			oplogCollection = oplogDatabase.getCollection(OPLOG);

			BasicDBObject where = new BasicDBObject();
			where.put("ns", database + "." + collection);
			tailedCursor = oplogCollection.find(where);
			tailedCursor.addOption(Bytes.QUERYOPTION_TAILABLE);
			tailedCursor.addOption(Bytes.QUERYOPTION_AWAITDATA);
			tailedCursor.skip(tailedCursor.size());
		}

		/**
		 * Run method for the TailedCursorThread. This will check for changes within the cursor and calls the onMessage method of its subscriber.
		 **/
		@Override
		public void run() {
			try {
				while (!Thread.interrupted()) {
					while (tailedCursor.hasNext()) {
						OplogEntry entry = new OplogEntry(
								(DBObject) tailedCursor.next());
						MongoOperation operation = entry.getOperation();

						for (BlackboardSubscription sub : subscriptions) {
							if (sub.getOperation().equals(operation)) {
								sub.getSubscriber().onMessage(operation, entry);
							}
						}
					}
				}
			} catch (MongoInterruptedException ex) {
			}
		}
	}
}