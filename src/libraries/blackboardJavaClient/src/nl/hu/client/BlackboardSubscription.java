/**
 * @file BlackboardSubscription.java
 * @brief Basic object for storing blackboard subscriptions.
 * @date Created: 2012-04-04
 *
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
 **/

package nl.hu.client;

/**
 * Object representing a subscriptiong to a blackboard event.
 *
 */
public class BlackboardSubscription {
	/**
	 * @var MongoOperation operation
	 * The operation the subscriber wishes to respond to.
	 */
	private MongoOperation operation;
	/**
	 * @var BlackboardSubscriber subscriber
	 * The object that should be called when the specifiedk operation occurs.
	 */
	private BlackboardSubscriber subscriber;
	
	/**
	 * Creates a BlackboardSubscriber object.
	 * 
	 * @param operation The operation the subscriber wishes to respond to.
	 * @param subscriber The object that should be called when the specifiedk operation occurs.
	 */
	public BlackboardSubscription(MongoOperation operation, BlackboardSubscriber subscriber) {
		this.operation = operation;
		this.subscriber = subscriber;
	}

	/**
	 * Returns the operation associated with this subscription.
	 * @return The operation associated with this subscription.
	 */
	public MongoOperation getOperation() {
		return operation;
	}

	/**
	 * Returns the subscriber for this subscription.
	 * @return The subscriber for this subscription.
	 */
	public BlackboardSubscriber getSubscriber() {
		return subscriber;
	}

	/**
	 * Computes the hashcode for this object.
	 * @return Hashcode for this object.
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((operation == null) ? 0 : operation.hashCode());
		result = prime * result
				+ ((subscriber == null) ? 0 : subscriber.hashCode());
		return result;
	}

	/**
	 * Checks whether two BlackboardSubscription objects are equal.
	 * Two objects are regarded as equals when both its operation and subscriber are equal.
	 * @return Whether or not the two objects are equal.
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		BlackboardSubscription other = (BlackboardSubscription) obj;
		if (operation != other.operation)
			return false;
		if (subscriber == null) {
			if (other.subscriber != null)
				return false;
		} else if (!subscriber.equals(other.subscriber))
			return false;
		return true;
	}
}
