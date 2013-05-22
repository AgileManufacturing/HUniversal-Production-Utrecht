/**
 * @file rexos/libraries/blackboard_client/BasicOperationSubscription.java
 * @brief Object representing a subscription to one of the basic MongoDB operations.
 * @date Created: 2013-04-08
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

import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

/**
 * Object representing a subscription to one of the basic MongoDB operations.
 *
 **/
public class BasicOperationSubscription extends BlackboardSubscription {
	/**
	 * @var MongoOperation operation
	 * The operation the subscriber wishes to respond to.
	 **/
	private MongoOperation operation;
	
	/**
	 * Creates a BlackboardSubscriber object.
	 * 
	 * @param operation The operation the subscriber wishes to respond to.
	 * @param subscriber The object that should be called when the specified operation occurs.
	 **/
	public BasicOperationSubscription(MongoOperation operation, BlackboardSubscriber subscriber) {
		super(subscriber);
		this.operation = operation;
	}

	/**
	 * Returns the operation associated with this subscription.
	 * @return The operation associated with this subscription.
	 **/
	public MongoOperation getOperation() {
		return operation;
	}

	/**
	 * Computes the hashcode for this object.
	 * @return Hashcode for this object.
	 **/
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result
				+ ((operation == null) ? 0 : operation.hashCode());
		return result;
	}

	/**
	 * Checks whether two BlackboardSubscription objects are equal.
	 * @param obj The other object.
	 * @return Whether or not the two objects are equal.
	 **/
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		BasicOperationSubscription other = (BasicOperationSubscription) obj;
		if (operation != other.operation)
			return false;
		return true;
	}

	/**
	 * @see rexos.libraries.blackboard_client.BlackboardSubscription#getQuery()
	 **/
	@Override
	public DBObject getQuery() {
		return QueryBuilder.start(OplogEntry.OPERATION_FIELD).is(operation.getOpCode()).get();
	}

	/**
	 * @see rexos.libraries.blackboard_client.BlackboardSubscription#matchesWithEntry(com.mongodb.DBObject)
	 **/
	@Override
	public boolean matchesWithEntry(OplogEntry entry) {
		return entry.getOperation() == operation;
	}
}
