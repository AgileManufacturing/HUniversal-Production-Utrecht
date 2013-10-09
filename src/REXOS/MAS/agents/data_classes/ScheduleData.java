/**
 * @file rexos/mas/data/ScheduleData.java
 * @brief Data object for storing scheduling data.
 * @date Created: 11 apr. 2013
 *
 * @author Peter Bonnema
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
package agents.data_classes;

import java.io.Serializable;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * Instances of this class contain schedule data with the start time,
 * duration and the deadline of a <code>ProductionStep</code> in timeslots.
 *
 */
public class ScheduleData implements MongoSaveable, Serializable {
	/**
	 * @var long serialVersionUID
	 * The serialVersionUID for this class.
**/
	private static final long serialVersionUID = 8380622365668923276L;
	
	/**
	 * @var int startTime
	 * The start time.
	 **/
	private long startTime;
	
	/**
	 * @var int duration
	 * The duration.
	 **/
	private long duration;
	
	/**
	 * @var int deadline
	 * The deadline.
	 **/
	private long deadline;
	
	/**
	 * Creates a new <code>ScheduleData</code> leaving <code>startTime</code>, <code>duration</code> and <code>deadline</code> uninitialized.
	 */
	public ScheduleData() {
		startTime = -1;
		duration = -1;
		deadline = -1;
	}
	
	/**
	 * Constructs a ScheduleData object with the data contained in the specified BasicDBObject.
	 * @param object The DBObject containing the data for this ScheduleData object.
	 */
	public ScheduleData(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * Creates a new <code>ScheduleData</code> with the specified start time, duration and deadline.
	 * 
	 * @param startTime When execution of the <code>ProductionStep</code> with this <code>ScheduleData</code> should start.
	 * @param duration How long it will take to complete the <code>ProductionStep</code>.
	 * @param deadline The <code>ProductionStep</code> with this <code>ScheduleData</code> should be finished after this timeslot.
	 */
	public ScheduleData(int startTime, int duration, int deadline) {
		this.startTime = startTime;
		this.duration = duration;
		this.deadline = deadline;
	}

	/**
	 * @see data.DBSaveable#toBasicDBObject()
	 **/
	@Override
	public BasicDBObject toBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start()
				.add("startTime", startTime)
				.add("duration", duration)
				.add("deadline", deadline).get();
	}

	/**
	 * @see data.DBSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 **/
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		BasicDBObject copy = (BasicDBObject) object.copy();
		this.startTime = copy.getLong("startTime", -1);
		copy.remove("startTime");
		this.duration = copy.getLong("duration", -1);
		copy.remove("duration");
		this.deadline = copy.getLong("deadline", -1);
		copy.remove("deadline");
		if(!copy.isEmpty()){
			throw new IllegalArgumentException();
		}
	}

	/**
	 * Returns the start time for this object.
	 * @return The start time
	 **/
	public long getStartTime() {
		return startTime;
	}

	/**
	 * Sets the start time for this object.
	 * @param startTime the start time to set
	 */
	public void setStartTime(long startTime) {
		this.startTime = startTime;
	}

	/**
	 * Returns the duration for this object.
	 * @return the duration
	 */
	public long getDuration() {
		return duration;
	}

	/**
	 * Sets the duration for this object.
	 * @param duration the duration to set
	 */
	public void setDuration(long duration) {
		this.duration = duration;
	}

	/**
	 * Returns the deadline for this object.
	 * @return the deadline
	 */
	public long getDeadline() {
		return deadline;
	}

	/**
	 * Sets the deadline for this object.
	 * @param deadline the deadline to set
	 */
	public void setDeadline(long deadline) {
		this.deadline = deadline;
	}

	/**
	 * @see java.lang.Object#toString()
	 **/
	@Override
	public String toString() {
		return String.format(
				"ScheduleData [startTime=%s, duration=%s, deadline=%s]",
				startTime, duration, deadline);
	}
}
