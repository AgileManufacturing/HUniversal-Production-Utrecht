/**
 * @file rexos/libraries/blackboard_client/MongoDBConnection.java
 * @brief Helper class for managing Mongo connections.
 * @date Created: 2012-04-05
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

package rexos.libraries.blackboard_client;

import java.util.Hashtable;

import com.mongodb.Mongo;
import com.mongodb.MongoException;
import com.mongodb.ServerAddress;
import com.mongodb.WriteConcern;

/**
 * Helper class for managing Mongo connections.
 **/
public class MongoDBConnection {
	/**
	 * @var Hashtable<ServerAddress, MongoDBConnection> databaseConnections
	 * Hashmap storing the active Mongo connection for each host.
	 **/
	private static Hashtable<ServerAddress, MongoDBConnection> databaseConnections;
	
	/**
	 * Initializes the databaseConnections Hashtable.
	 **/
	static {
		databaseConnections = new Hashtable<>();
	}
	
	/**
	 * @var Mongo mongoClient
	 * The Mongo object representing this connection.
	 **/
	private Mongo mongoClient;
	
	/**
	 * @var ServerAddress address
	 * The host address to which this client is connected.
	 **/
	private ServerAddress address;
	
	/**
	 * Creates a new Mongo client for the specified address.
	 * @param address The ServerAddress where the host resides.
	 * @throws GeneralMongoException Connecting to the database server failed.
	 **/
	private MongoDBConnection(ServerAddress address) throws GeneralMongoException {
		try {
			mongoClient = new Mongo(address);
			mongoClient.setWriteConcern(WriteConcern.SAFE);
		} catch (MongoException mongoException) {
			throw new GeneralMongoException("A mongo exception occurred while connecting.", mongoException);
		}
		this.address = address;
	}
	
	/**
	 * Returns a {@link MongoDBConnection} instance for the specified host.
	 * @param address The ServerAddress where the host resides.
	 * @return A {@link MongoDBConnection} instance for the specified host.
	 * @throws MongoConnectionException Connecting to the database server failed.
	 **/
	public static MongoDBConnection getInstanceForHost(ServerAddress address) throws GeneralMongoException {
		if (!databaseConnections.containsKey(address)) {
			databaseConnections.put(address, new MongoDBConnection(address));
		}
		
		return databaseConnections.get(address);
	}
	
	/**
	 * Returns the Mongo client for this connection.
	 * @return The Mongo client for this connection.
	 **/
	public Mongo getMongoClient() {
		return mongoClient;
	}
	
	/**
	 * Returns the ServerAddress for this connection.
	 * @return The ServerAddress for this connection.
	 **/
	public ServerAddress getServerAddress() {
		return address;
	}
}
