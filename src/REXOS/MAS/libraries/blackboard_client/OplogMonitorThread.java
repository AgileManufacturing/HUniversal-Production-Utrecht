/**
 * @file rexos/libraries/blackboard_client/OplogMonitorThread.java
 * @brief Class for the tailed oplog cursor thread within the client
 * @date Created: 18 apr. 2013
 *
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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
 **/
package libraries.blackboard_client;

import java.util.ArrayList;

import libraries.blackboard_client.data_classes.BlackboardSubscription;
import libraries.blackboard_client.data_classes.OplogEntry;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import com.mongodb.Bytes;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.Mongo;
import com.mongodb.MongoException;
import com.mongodb.MongoInterruptedException;

/**
 * Class for the tailed oplog cursor thread within the client
 **/
class OplogMonitorThread extends Thread {
	
	/**
	 * @var int POLL_INTERVAL
	 * The interval between two checks for new documents in milliseconds. Note that this interval is not used
	 * when there are multiple documents available. In this case all documents will be retrieved after which
	 * the thread will sleep for this interval when no more documents are available.
	 **/
	private static final int POLL_INTERVAL = 100;

	private DB database;
	/**
	 * @var DBCursor tailedCursor
	 * Tailed cursor for this thread.
	 **/
	private DBCursor tailedCursor;
	
	/**
	 * @var BlackboardSubscription subscriptions[]
	 * Link between subscribed topic name and MongoDbs BasicDBObjects
	 **/
	private BlackboardSubscription[] subscriptions;
	
	/**
	 * @var OplogCallbackThread callbackThread
	 * Thread used for executing callbacks.
	 */
	OplogCallbackThread callbackThread;
	
	/**
	 * Constructor of OplogMonitorThread.
	 * @param mongo The Mongo database connection that should be used.
	 * @param oplogDBName The database in which the oplog collection resides.
	 * @param oplogCollectionName The name of the oplog collection.
	 * @param query The query that will be used in the tailed cursor.
	 **/
	public OplogMonitorThread(Mongo mongo, String oplogDBName, String oplogCollectionName, DBObject query) {
		database = mongo.getDB(oplogDBName);
		DBCollection collection = database.getCollection(oplogCollectionName);
		
		tailedCursor = collection.find(query);
		tailedCursor.addOption(Bytes.QUERYOPTION_TAILABLE);
		tailedCursor.addOption(Bytes.QUERYOPTION_AWAITDATA);
		tailedCursor.skip(tailedCursor.size());
		callbackThread = new OplogCallbackThread();
	}
	
	/**
	 * Constructs a tailed cursor for the specified query on the oplog collection.
	 * This constructor should be used when user authentication is required.
	 * 
	 * @param mongo The Mongo database connection that should be used.
	 * @param oplogDBName The database in which the oplog collection resides.
	 * @param oplogCollectionName The name of the oplog collection.
	 * @param username Username that will be used to authenticate with the oplog database. This user should have read access.
	 * @param password The password belonging to the specified user.
	 * @param query The query that will be used in the tailed cursor.
	 **/
	public OplogMonitorThread(Mongo mongo, String oplogDBName, String oplogCollectionName,
		String username, String password, DBObject query) {
		database = mongo.getDB(oplogDBName);
		database.authenticate(username, password.toCharArray());
		DBCollection collection = database.getCollection(oplogCollectionName);
		
		tailedCursor = collection.find(query);
		tailedCursor.addOption(Bytes.QUERYOPTION_TAILABLE);
		tailedCursor.addOption(Bytes.QUERYOPTION_AWAITDATA);
		tailedCursor.skip(tailedCursor.size());
		callbackThread = new OplogCallbackThread();
	}
	
	/**.requestStart()
	 * Sets the subscriptions that are used for the query of this oplog monitor.
	 * @param subscriptions ArrayList containing all the subscriptions this monitor will subscribe to.
	 **/
	public void setSubscriptions(ArrayList<BlackboardSubscription> subscriptions) {
		this.subscriptions = new BlackboardSubscription[subscriptions.size()];
		subscriptions.toArray(this.subscriptions);
	}

	/**
	 * Run method for the TailedCursorThread.
	 * This will check for changes within the cursor and calls the onMessage method of its subscriber.
	 */
	@Override
	public void run() {
		try{
			do {
				while (!Thread.interrupted() && tailedCursor.hasNext()) {
					OplogEntry entry = new OplogEntry(tailedCursor.next());
	
					for (BlackboardSubscription sub : subscriptions) {
						if (sub.matchesWithEntry(entry)) {
							callbackThread.addCallback(sub.getSubscriber(), entry);
						}
					}
				}
				Thread.sleep(POLL_INTERVAL);
			} while (!Thread.interrupted() && tailedCursor.getCursorId() != 0);
		} catch (MongoInterruptedException | MongoException.CursorNotFound | InterruptedException ex) {
			/*
			 * MongoInterruptedException is thrown by Mongo when interrupt is called while blocking on the
			 * tailedCursor's hasNext method. When this happens, return from the run method to kill the thread.
			 */
			
			/*
			 * MongoException.CursorNotFound indicates the cursor was killed while blocking on hasNext.
			 * We purposely kill the cursor when the OplogMonitorThread is interrupted, thus expect this to happen.
			 */
			Logger.log(LogLevel.ERROR,"OplogMonitorThread ending due to %s:\n%s\n", ex.getClass().getName(), ex.getMessage());
		} finally {
			try {
				if (tailedCursor != null) {
					tailedCursor.close();
				}
			} catch (Throwable t) {
				// If closing the cursor throws something, it's most likely not something we can fix.
				Logger.log(LogLevel.ERROR, "%s thrown while closing cursor:\n%s\n", t.getClass().getName(), t.getMessage());
			}
			//
		}
		callbackThread.shutdown();
	}
}
