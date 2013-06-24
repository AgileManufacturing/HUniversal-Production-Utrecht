/**
 * @file StateEntry.java
 * @brief An entry for the state blackboard
 * @date Created: 24 jun. 2013
 *
 * @author Hessel Meulenbeld
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright � 2013, HU University of Applied Sciences Utrecht.
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
package rexos.mas.data;

import com.mongodb.BasicDBObject;

/**
 * entry for the state blackboard
 */
public class StateEntry implements MongoSaveable {
	/**
	 * enum for the states of an equiplet
	 */
	public static enum State{
		/**
		 * safe state of the equiplet
		 */
		SAFE(0),
		/**
		 * standby state of the equiplet
		 */
		STANDBY(3),
		/**
		 * normal state of the equiplet
		 */
		NORMAL(6);
		
		/**
		 * @var int value
		 * 		The value of the state
		 */
		private int value;
		
		/**
		 * constructor for the state
		 * @param value The value of the state
		 */
		private State(int value){
			this.value = value;
		}
	}
	
	/**
	 * @var String name
	 * 		The name of the equiplet
	 */
	private String name;
	
	/**
	 * @var State desired_state
	 * 		The state desired for the equiplet
	 */
	private State desiredState;
	
	/**
	 * constructor for the state entry
	 * @param name the name of the equiplet
	 * @param desiredState the desiredState for the equiplet
	 */
	public StateEntry(String name, State desiredState){
		this.name = name;
		this.desiredState = desiredState;
	}
	
	/**
	 * @see rexos.mas.data.MongoSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("name", name);
		object.put("desiredState", desiredState.value);
		return object;
	}

	/**
	 * @see rexos.mas.data.MongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) throws IllegalArgumentException {
		BasicDBObject copy = (BasicDBObject) object.copy();
		this.name = (String) copy.remove("name");
		this.desiredState = State.valueOf((String) copy.remove("desiredState"));
		if(!copy.isEmpty()){
			throw new IllegalArgumentException();
		}
	}

}
