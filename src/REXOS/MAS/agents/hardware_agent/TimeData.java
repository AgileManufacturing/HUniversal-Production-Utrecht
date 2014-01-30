/**
 * @file src/REXOS/MAS/agents/hardware_agent/TimeData.java
 * @brief Provides methods to get and set the TimeData.
 * @date Created: 2013-04-02
 *
 * @author Thierry Gerritse
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
package agents.hardware_agent;

import libraries.blackboard_client.data_classes.MongoSaveable;

import com.mongodb.BasicDBObject;

/**
 * TimeData class containing the time data of an EquipletStep.
 */
public class TimeData implements MongoSaveable {
	
	/**
	 * @var long duration
	 * The duration in timeslots
	 */
	private long duration;

	/**
	 * Constructor
	 * @param duration The duration in timeslots
	 */
	public TimeData(long duration) {
		this.duration = duration;
	}

	/**
	 * Constructor
	 * @param object The BasicDBObject to fill this class with.
	 */
	public TimeData(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * getter for the duration.
	 * @return duration
	 */
	public long getDuration() {

		return duration;

	}

	/**
	 * setter to set the duration
	 * @param duration The duration in timeslots
	 */
	public void setDuration(long duration) {

		this.duration = duration;

	}

	/**
	 * @see rexos.mas.data.MongoSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("duration", duration);
		return object;
	}

	/**
	 * @see rexos.mas.data.MongoSaveable#fromBasicDBObject(BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		BasicDBObject copy = (BasicDBObject) object.copy();
		duration = (long) copy.remove("duration");
		if(!copy.isEmpty()){
			throw new IllegalArgumentException();
		}
	}
}
