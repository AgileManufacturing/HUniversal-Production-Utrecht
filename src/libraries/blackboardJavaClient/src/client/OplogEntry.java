/**
 * @file OplogEntry.java
 * @brief Representation of a document in the oplog collection.
 * @date Created: 2012-04-04
 * 
 * @author Jan-Willem Willebrands
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package libraries.blackboardJavaClient.src.client;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class OplogEntry{
	private static final String NAMESPACE_FIELD = "ns";
	@SuppressWarnings("unused")
	private static final String TIMESTAMP_FIELD = "ts";
	private static final String OPERATION_FIELD = "op";
	@SuppressWarnings("unused")
	private static final String UID_FIELD = "h";
	private static final String UPDATE_DOC_FIELD = "o";
	private static final String UPDATE_CRITERIA_FIELD = "o2";
	private DBObject oplogEntry;

	public OplogEntry(DBObject oplogEntry){
		this.oplogEntry = oplogEntry;
	}

	public MongoOperation getOperation(){
		Object obj = oplogEntry.get(OPERATION_FIELD);
		return MongoOperation.get((String) obj);
	}

	public String getNamespace(){
		return oplogEntry.get(NAMESPACE_FIELD).toString();
	}

	public String getUpdateDocument(){
		return oplogEntry.get(UPDATE_DOC_FIELD).toString();
	}

	public String getUpdateCriteria(){
		return oplogEntry.get(UPDATE_CRITERIA_FIELD).toString();
	}

	@Override
	public String toString(){
		return oplogEntry.toString();
	}

	public ObjectId getTargetObjectId(){
		Object targetObj = oplogEntry.get("o2");
		if (targetObj == null){
			targetObj = oplogEntry.get("o");
		}
		ObjectId id = null;
		if (targetObj instanceof BasicDBObject){
			id = ((BasicDBObject) targetObj).getObjectId("_id");
		}
		return id;
	}
}
