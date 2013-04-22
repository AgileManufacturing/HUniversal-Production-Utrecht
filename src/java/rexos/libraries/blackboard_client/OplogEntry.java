/**
 * @file OplogEntry.java
 * @brief Representation of a document in the oplog collection.
 * @date Created: 2012-04-04
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
 * 
 **/

package rexos.libraries.blackboard_client;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import org.bson.types.ObjectId;

/**
 * Utility class for handling an oplog entry.
 **/
public class OplogEntry {
	
	/**
	 * @var String NAMESPACE_FIELD
	 * Name of the namespace field within the oplog entry.
	 **/
	public static final String NAMESPACE_FIELD = "ns";
	
	/**
	 * @var String TIMESTAMP_FIELD
	 * Name of the timestamp field within the oplog entry.
	 **/
	public static final String TIMESTAMP_FIELD = "ts";
	
	/**
	 * @var String OPERATION_FIELD
	 * Name of the operation field within the oplog entry.
	 **/
	public static final String OPERATION_FIELD = "op";
	
	/**
	 * @var String UID_FIELD
	 * Name of the uid field within the oplog entry.
	 **/
	public static final String UID_FIELD = "h";
	
	/**
	 * @var String UPDATE_DOC_FIELD
	 * Name of the update document field within the oplog entry.
	 **/
	public static final String UPDATE_DOC_FIELD = "o";
	
	/**
	 * @var String UPDATE_CRITERIA_FIELD
	 * Name of the update criteria field within the oplog entry.
	 **/
	public static final String UPDATE_CRITERIA_FIELD = "o2";
	
	/**
	 * @var DBObject oplogEntry
	 * Internal DBObject representation of the oplog entry.
	 **/
	private DBObject oplogEntry;

	/**
	 * Construct an OplogEntry object based on the specified entry.
	 * @param oplogEntry The DBObject containing the oplog data.
	 **/
	public OplogEntry(DBObject oplogEntry) {
		this.oplogEntry = oplogEntry;
	}
	
	/**
	 * Returns the operation of this oplog entry.
	 * @return the operation of this oplog entry.
	 **/
	public MongoOperation getOperation() {
		Object obj = oplogEntry.get(OPERATION_FIELD);
		return MongoOperation.get((String)obj);
	}
	
	/**
	 * Returns the namespace of this oplog entry.
	 * @return the namespace of this oplog entry.
	 **/
	public String getNamespace() {
		return oplogEntry.get(NAMESPACE_FIELD).toString();
	}
	
	/**
	 * Returns the update query (i.e. the query that was used to update the target documents) of this oplog entry.
	 * @return the update query of this oplog entry.
	 **/
	public DBObject getUpdateDocument() { 
		return (DBObject)oplogEntry.get(UPDATE_DOC_FIELD);
	}
	
	/**
	 * Returns the update criteria (i.e. the query used to match target documents) of this oplog entry.
	 * @return the update criteria of this oplog entry.
	 **/
	public DBObject getUpdateCriteria() {
		return (DBObject)oplogEntry.get(UPDATE_CRITERIA_FIELD);
	}

	/**
	 * Returns a JSON serialization of the oplog entry.
	 * @return A JSON serialization of the oplog entry.
	 **/
	public String toString() {
		return oplogEntry.toString();
	}
	
	/**
	 * Attempts to retrieve the ObjectId for the target document of the operation.
	 * @return ObjectId representing the target document or null.
	 **/
	public ObjectId getTargetObjectId() {
		Object targetObj = oplogEntry.get("o2");
		if (targetObj == null) {
			targetObj = oplogEntry.get("o");
		}
		
		ObjectId id = null;
		if (targetObj instanceof BasicDBObject) {
			id = ((BasicDBObject)targetObj).getObjectId("_id");
		}

	
		return id;
	}
}
