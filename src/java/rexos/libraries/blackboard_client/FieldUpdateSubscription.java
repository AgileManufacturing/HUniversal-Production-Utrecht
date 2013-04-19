/**
 * @file FieldUpdateSubscription.java
 * @brief Represents a subscription to a specified set of changes for a certain field.
 * @date Created: 10 apr. 2013
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

import java.util.EnumSet;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

/**
 * Represents a subscription to a specified set of changes for a certain field.
 **/
public class FieldUpdateSubscription extends BasicOperationSubscription {
	/**
	 * Representation of the update operations that appear in the oplog.
	 * Although there are more update operations, they are all simplified to set and unset operations when written to the oplog.
	 **/
	public static enum MongoUpdateLogOperation {
		
		/**
		 * Represents a set operation in MongoDB.
		 **/
		SET("$set"),
		
		/**
		 * Represents an unset operation in MongoDB.
		 **/
		UNSET("$unset"),
		
		/**
		 * Represents an update operation that replaces the contents of the entire object.
		 * This is used when no update operator is specified in the update query, e.g.: update({_id:12}, {data : "new"})
		 **/
		REPLACE(null);
		
		/**
		 * @var String operatorString
		 * The String (as it appears in the oplog) representing this operation.
		 **/
		private final String operatorString;
		
		/**
		 * Constructs the object with the specified operatorString.
		 * @param operatorString A string representation of the operation as it would appear in the oplog.
		 **/
		private MongoUpdateLogOperation(String operatorString) {
			this.operatorString = operatorString;
		}
		
		/**
		 * Returns the operator string, which is a string representation of the operation as it would appear in the oplog.
		 * @return The operator string.
		 **/
		public String getOperatorString() {
			return operatorString;
		}
	}
	
	/**
	 * @var String fieldName
	 * The name of the field that should be monitored for updates.
	 **/
	private String fieldName;
	
	/**
	 * @var Set<MongoUpdateLogOperator> subscribedOperations
	 * The update operations that should be monitored for the specified field.
	 **/
	EnumSet<MongoUpdateLogOperation> subscribedOperations;
	
	/**
	 * Creates a subscription for updates to the specified field.
	 * The desired operations to monitor should be added after construction using the addOperation method.
	 * @param fieldName The name of the field that should be monitored for updates.
	 * @param subscriber The object that should be called when the specified operation occurs.
	 **/
	public FieldUpdateSubscription(String fieldName, BlackboardSubscriber subscriber) {
		super(MongoOperation.UPDATE, subscriber);
		subscribedOperations = EnumSet.noneOf(MongoUpdateLogOperation.class);
		this.fieldName = fieldName;
	}
	
	/**
	 * Adds a parameter to the monitored set.
	 * @param operation The operation to add.
	 **/
	public void addOperation(MongoUpdateLogOperation operation) {
		subscribedOperations.add(operation);
	}
	
	/**
	 * Removes an operator from the monitored set.
	 * @param operation The operation to remove.
	 **/
	public void removeOperator(MongoUpdateLogOperation operation) {
		subscribedOperations.remove(operation);
	}
	
	/**
	 * Generates the field string that can be used in an exists query.
	 * 
	 * Example output for watching field data for SET changes, assuming UPDATE_DOC_FIELD is "o":
	 *  	"o.$set.data"
	 *  
	 * @param operation The operation to generate the string for.
	 * @return The field string that can be used in an exists query.
	 */
	private String createOplogUpdateFieldString(MongoUpdateLogOperation operation) {
		StringBuilder updateField = new StringBuilder(OplogEntry.UPDATE_DOC_FIELD).append(".");
		String operator = operation.getOperatorString();
		if (operator != null) {
			updateField.append(operator).append(".");
		}
		
		return updateField.append(fieldName).toString();
	}
	
	/**
	 * @see rexos.libraries.blackboard_client.BasicOperationSubscription#getQuery()
	 **/
	@Override
	public DBObject getQuery() {
		DBObject[] operationQueries = new DBObject[subscribedOperations.size()];
		int i = 0;
		for (MongoUpdateLogOperation operator : subscribedOperations) {
			operationQueries[i++] = QueryBuilder.start(createOplogUpdateFieldString(operator)).exists(true).get();
		}
		return QueryBuilder.start().and(
				super.getQuery(),
				QueryBuilder.start().or(operationQueries).get()).get();
	}
	
	/**
	 * @see rexos.libraries.blackboard_client.BasicOperationSubscription#matchesWithEntry(rexos.libraries.blackboard_client.OplogEntry)
	 **/
	@Override
	public boolean matchesWithEntry(OplogEntry entry) {
		if (!super.matchesWithEntry(entry)) {
			return false;
		}
		
		DBObject updateDoc = entry.getUpdateDocument();
		for (MongoUpdateLogOperation operator : subscribedOperations) {
			String operatorString = operator.getOperatorString();
			if (operatorString != null) {
				if (updateDoc.containsField(operator.getOperatorString())) {
					DBObject operatorObj = (DBObject)updateDoc.get(operator.getOperatorString());
					if (operatorObj.containsField(fieldName)) {
						return true;
					}
				}
			} else if (updateDoc.containsField(fieldName)) {
				return true;
			}

		}
		
		return false;
	}
}
