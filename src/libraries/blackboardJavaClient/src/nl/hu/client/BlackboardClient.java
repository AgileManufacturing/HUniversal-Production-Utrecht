﻿/**
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
 * Copyright © 2012-2013, HU University of Applied Sciences Utrecht.
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

package libraries.blackboardJavaClient.src.nl.hu.client;

import java.util.ArrayList;
import java.util.List;

import org.bson.types.ObjectId;

import com.mongodb.Bytes;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.Mongo;
import com.mongodb.MongoException;
import com.mongodb.MongoInterruptedException;
import com.mongodb.QueryBuilder;
import com.mongodb.ServerAddress;
import com.mongodb.WriteResult;
import com.mongodb.util.JSON;
import com.mongodb.util.JSONParseException;

/**
 * Client class for a mongodb blackboard.
 **/
public class BlackboardClient {
	/**
	 * @var String OPLOG
	 * Operation log collection name of MongoDB.
	 **/
	private static final String OPLOG_COLLECTION_NAME = "oplog.rs";

	/**
	 * @var String LOCAL
	 * Local database name of MongoDB.
	 **/
	private static final String OPLOG_DATABASE_NAME = "local";

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
	 * @var TailedCursorThread tcThread
	 * Thread for tracking tailable cursor on operation log of MongoDB
	 **/
	private OplogMonitorThread oplogMonitorThread;
	
	/**
	 * Constructor of BlackboardClient.
	 *
	 * @param host The mongoDB host.
	 **/
	public BlackboardClient(String host) {
		try {
			this.subscriptions = new ArrayList<>();
			this.mongo = MongoDBConnection.getInstanceForHost(new ServerAddress(host)).getMongoClient();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Constructor of BlackboardClient.
	 *
	 * @param host The ip of the MongoDB host.
	 * @param port The port of the MongoDB host.
	 **/
	public BlackboardClient(String host, int port) {
		try {
			this.subscriptions = new ArrayList<>();
			this.mongo = MongoDBConnection.getInstanceForHost(new ServerAddress(host, port)).getMongoClient();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Utility function for parsing JSON that catches the runtime JSON exception and throws an InvalidJSONException instead.
	 * @param jsonString The JSON string that needs to be parsed to a DBObject.
	 * @return The DBObject parsed from the JSON string.
	 * @throws InvalidJSONException An error exists within the JSON.
	 **/
	public static DBObject parseJSONWithCheckException(String jsonString) throws InvalidJSONException {
		DBObject obj = null;
		try {
			obj = (DBObject)JSON.parse(jsonString);
		} catch (JSONParseException ex) {
			throw new InvalidJSONException(ex);
		}
		
		return obj;
	}
	
	/**
	 * Sets the database to use.
	 *
	 * @param database The database to load from MongoDB.
	 * @throws InvalidDBNamespaceException Database name cannot be empty.
	 **/
	public void setDatabase(String database) throws InvalidDBNamespaceException {
		if (database == null || database.isEmpty()) {
			throw new InvalidDBNamespaceException("Database name cannot be empty.");
		}
		currentCollection = null;
		currentDatabase = mongo.getDB(database);
	}
	
	/**
	 * Sets the database to use and connects using the specified username and password.
	 * @throws InvalidDBNamespaceException Database name cannot be empty.
	 * @throws DBAuthException Authentication failed.
	 **/
	public void setDatabase(String database, String user, String password) throws InvalidDBNamespaceException, DBAuthException {
		setDatabase(database);
		if (!currentDatabase.authenticate(user, password.toCharArray())) {
			throw new DBAuthException("The provided username and password combination is incorrect.");
		}
	}

	/**
	 * Sets the collection to use.
	 *
	 * @param collection The collection to load from MongoDB.
	 * @throws InvalidDBNamespaceException No database has been selected.
	 **/
	public void setCollection(String collection) throws InvalidDBNamespaceException {
		if (currentDatabase == null) {
			throw new InvalidDBNamespaceException("No database selected");
		}
		currentCollection = currentDatabase.getCollection(collection);
	}

	/**
	 * Inserts a document into the currently selected collection.
	 * 
	 * @param obj DBObject representing the document to be inserted.
	 * @return ObjectId of the inserted object.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public ObjectId insertDocument(DBObject obj) throws InvalidDBNamespaceException {
		if (currentCollection == null) {
			throw new InvalidDBNamespaceException("No collection has been selected.");
		}
		currentCollection.insert(obj);
		return ObjectId.massageToObjectId(obj.get("_id"));
	}
	
	/**
	 * Inserts document into the currently selected collection using JSON format.
	 *
	 * @param json JSON String representing the document to be inserted.
	 * @return ObjectId of the inserted object.
	 * @throws InvalidJSONException The provided JSON contains errors.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public ObjectId insertDocument(String json) throws InvalidJSONException, InvalidDBNamespaceException {
		DBObject obj = parseJSONWithCheckException(json);
		return insertDocument(obj);
	}

	/**
	 * Removes all documents matching the provided query from the currently selected collection.
	 * 
	 * @param query DBObject representing the query used for deleting documents.
	 * @return The amount of records that have been removed.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public int removeDocuments(DBObject query) throws InvalidDBNamespaceException {
		if (currentCollection == null) {
			throw new InvalidDBNamespaceException("No collection has been selected.");
		}
		
		WriteResult res = currentCollection.remove(query);
		return res.getN();
	}
	
	/**
	 * Removes all documents matching the provided query from the currently selected collection.
	 *
	 * @param queryAsJSON JSON serialization of an Object 
	 * @return The amount of records that have been removed.
	 * @throws InvalidJSONException The provided JSON contains errors.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public int removeDocuments(String queryAsJSON) throws InvalidJSONException, InvalidDBNamespaceException {
		DBObject query = parseJSONWithCheckException(queryAsJSON);
		return removeDocuments(query);
	}
	
	/**
	 * Retrieves the document corresponding to the given ObjectId.
	 *
	 * @param query ObjectId of the requested object.
	 * @return The object corresponding to the given id, or null if no such object was found.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public DBObject findDocumentById(ObjectId objId) throws InvalidDBNamespaceException {
		if (currentCollection == null) {
			throw new InvalidDBNamespaceException("No collection has been selected.");
		}
		
		DBObject found = currentCollection.findOne(QueryBuilder.start("_id").is(objId).get());
		return found;
	}
	
	/**
	 * Retrieves all documents matching the provided query from the currently selected collection.
	 *
	 * @param query DBObject representing the query.
	 * @return List of all documents matching the query.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public List<DBObject> findDocuments(DBObject query) throws InvalidDBNamespaceException {
		if (currentCollection == null) {
			throw new InvalidDBNamespaceException("No collection has been selected.");
		}
		
		List<DBObject> found = currentCollection.find(query).toArray();
		return found;
	}
	
	/**
	 * Retrieves all documents matching the provided query from the currently selected collection.
	 *
	 * @param queryAsJSON JSON string representing the query.
	 * @return List of all documents matching the query.
	 * @throws InvalidJSONException The provided JSON contains errors.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public List<DBObject> findDocuments(String queryAsJSON) throws InvalidJSONException, InvalidDBNamespaceException {
		DBObject query = parseJSONWithCheckException(queryAsJSON);
		return findDocuments(query);
	}
	
	/**
	 * Updates all documents matching the provided search query within the currently selected collection.
	 * Documents are updated according to the query specified in updateQuery.
	 * 
	 * @param searchQuery The query that should be used to select the target documents.
	 * @param updateQuery The query that should be used to update the target documents.
	 * @return The amount of documents that have been updated.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 * @throws Exception
	 **/
	public int updateDocuments(DBObject searchQuery, DBObject updateQuery) throws InvalidDBNamespaceException {
		if (currentCollection == null) {
			throw new InvalidDBNamespaceException("No collection has been selected.");
		}
		
		WriteResult res = currentCollection.update(searchQuery, updateQuery);
		return res.getN();
	}
	
	/**
	 * Updates all documents matching the provided search query within the currently selected collection.
	 * Documents are updated according to the query specified in updateQuery.
	 *
	 * @param searchQueryAsJSON JSON serialization of the query that should be used to select the target documents.
	 * @param updateQueryAsJSON JSON serialization of the query that should be used to update the target documents.
	 * @return The amount of documents that have been updated.
	 * @throws InvalidJSONException The provided JSON contains errors.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 **/
	public int updateDocuments(String searchQueryAsJSON, String updateQueryAsJSON) throws InvalidJSONException, InvalidDBNamespaceException {
		DBObject searchQuery = parseJSONWithCheckException(searchQueryAsJSON);
		DBObject updateQuery = parseJSONWithCheckException(updateQueryAsJSON);
		
		return updateDocuments(searchQuery, updateQuery);
	}
	
	/**
	 * Subscribes to the specified CRUD operation in the current database and collection.
	 * 
	 * @param sub Specification of operation and callback object.
	 * @throws InvalidDBNamespaceException No collection has been selected.
	 * @return true if subscription was successful. false otherwise.
	 **/
	public boolean subscribe(BlackboardSubscription sub) throws InvalidDBNamespaceException {
		if (currentCollection == null) {
			throw new InvalidDBNamespaceException("No collection selected");
		}
		subscriptions.add(sub);
		
		// Attempt to create a new thread for the current subscriptions.
		return createNewMonitorThread();
	}
	
	/**
	 * Removes the specified subscription.
	 * 
	 * @param sub Subscription that should be removed.
	 **/
	public void unsubscribe(BlackboardSubscription sub) {
		subscriptions.remove(sub);
		
		if (subscriptions.size() > 0) {
			createNewMonitorThread();
		} else {
			oplogMonitorThread.interrupt();
		}
	}
	
	/**
	 * Closes and cleans up any open resources for this client.
	 * Should be called before disposing of the BlackboardClient object.
	 **/
	public void close() {
		
		// Stop the monitor thread.
		if (oplogMonitorThread != null) {
			oplogMonitorThread.interrupt();
		}
	}

	/**
	 * Attempts to create and start a new monitor thread.
	 * If creation of a new thread fails, the old thread will be kept alive.
	 * @return true if the new thread was created successfully, false otherwise.
	 **/
	private boolean createNewMonitorThread() {
		OplogMonitorThread newThread = null;
		
		// Create a query that will select all documents matching one of the queries of the subscribers within the
		// selected database/collection combination.
		DBObject[] subs = new DBObject[subscriptions.size()];
		for (int i = 0 ; i < subscriptions.size() ; ++i) {
			subs[i] = subscriptions.get(i).getQuery();
		}
		
		DBObject query = QueryBuilder.start("ns").is(currentDatabase.getName() + "." + currentCollection.getName())
				.and(subs).get();

		try {
			newThread = new OplogMonitorThread(OPLOG_DATABASE_NAME, OPLOG_COLLECTION_NAME, query);
		} catch (MongoException ex) {
			// This can happen when the database has not been configured properly and the Oplog collection does not exist.
			// Creating a tailed cursor on a non-existing collection throws a MongoException.
			return false;
		}
		
		if (oplogMonitorThread != null) {
			oplogMonitorThread.interrupt();
		}
		oplogMonitorThread = newThread;
		oplogMonitorThread.start();
		
		return true;
	}
	
	/**
	 * Class for tailable cursor thread within the client
	 **/
	private class OplogMonitorThread extends Thread {
		
		/**
		 * Tailed cursor for this thread.
		 **/
		private DBCursor tailedCursor;
		
		/**
		 * Constructor of TailedCursorThread.
		 **/
		public OplogMonitorThread(String oplogDBName, String oplogCollectionName, DBObject query) {
			DB database = mongo.getDB(oplogDBName);
			DBCollection collection = database.getCollection(oplogCollectionName);

			tailedCursor = collection.find(query);
			tailedCursor.addOption(Bytes.QUERYOPTION_TAILABLE);
			tailedCursor.addOption(Bytes.QUERYOPTION_AWAITDATA);
			tailedCursor.skip(tailedCursor.size());
		}

		/**
		 * Run method for the TailedCursorThread.
		 * This will check for changes within the cursor and calls the onMessage method of its subscriber.
		 **/
		@Override
		public void run() {
			try {
				while (!Thread.interrupted()) {
					while (tailedCursor.hasNext()) {
						OplogEntry entry = new OplogEntry(
								tailedCursor.next());
						MongoOperation operation = entry.getOperation();

						for (BlackboardSubscription sub : subscriptions) {
							if (sub.matchesWithEntry(entry)) {
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