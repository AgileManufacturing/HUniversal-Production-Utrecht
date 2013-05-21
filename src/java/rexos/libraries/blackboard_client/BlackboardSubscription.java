/**
 * @file rexos/libraries/blackboard_client/BlackboardSubscription.java
 * @brief Object representing a subscription to a blackboard event.
 * @date Created: 8 apr. 2013
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

import com.mongodb.DBObject;

/**
 * Object representing a subscription to a blackboard event.
 **/
public abstract class BlackboardSubscription {
	/**
	 * @var BlackboardSubscriber subscriber
	 * The object that should be called when the specified operation occurs.
	 **/
	private BlackboardSubscriber subscriber;
	
	/**
	 * Creates the query associated with this subscription.
	 * @return The query associated with this subscription.
	 **/
	public abstract DBObject getQuery();
	
	/**
	 * Returns whether or not the given object matches the query for this subscription.
	 * @param entry The oplog entry to check with the query.
	 * @return Whether or not the given object matches the query for this subscription.
	 **/
	public abstract boolean matchesWithEntry(OplogEntry entry);
	
	/**
	 * Constructs a BlackboardSubscription object for the given subscriber.
	 * @param subscriber The subscriber of this subscription.
	 **/
	public BlackboardSubscription(BlackboardSubscriber subscriber) {
		this.subscriber = subscriber;
	}
	
	/**
	 * Returns the subscriber for this subscription.
	 * @return The subscriber for this subscription.
	 **/
	public BlackboardSubscriber getSubscriber() {
		return subscriber;
	}
	
	/**
	 * Computes the hashcode for this object.
	 * @return Hashcode for this object.
	 **/
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((subscriber == null) ? 0 : subscriber.hashCode());
		return result;
	}

	/**
	 * Checks whether two BlackboardSubscription objects are equal.
	 * @return Whether or not the two objects are equal.
	 **/
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		BlackboardSubscription other = (BlackboardSubscription) obj;
		if (subscriber == null) {
			if (other.subscriber != null)
				return false;
		} else if (!subscriber.equals(other.subscriber))
			return false;
		return true;
	}
}
