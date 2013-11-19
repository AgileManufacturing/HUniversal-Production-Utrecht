/**
 * @file REXOS/MAS/libraries/schedule/data_classes/FreeTimeSlot.java
 * @brief Data object representing a freetimeslot in a schedule.
 * @date Created: 04 nov 2013
 * 
 * @author Roy Scheefhals
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
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


package libraries.schedule.data_classes;

import org.bson.types.ObjectId;

import libraries.blackboard_client.data_classes.MongoSaveable;

import com.mongodb.BasicDBObject;

/**
 * Data object representing a freetimeslot in a schedule. 
 * Implements the mongosavable for saving in blackboards.
 * @author Roy Scheefhals
 *
 */
public class FreeTimeSlot implements MongoSaveable{
	
	private TimeSlot timeSlot;
	
	/**
	 * @return the timeSlot
	 */
	public TimeSlot getTimeSlot() {
		return timeSlot;
	}

	/**
	 * Constructor for making a mongo blackboard entry to a FreeTimeSlot
	 * @param dbObject the monogoDB object
	 */
	public FreeTimeSlot(BasicDBObject dbObject){
		fromBasicDBObject(dbObject);
	}
	
	/**
	 * standard constructor
	 * @param startTimeSlot
	 * @param duration
	 */
	public FreeTimeSlot(long startTimeSlot, Long duration){
		this.timeSlot.setStartTimeSlot(startTimeSlot);
		this.timeSlot.setDuration(duration);
	}

	/**
	 * gets the starttime of this firsttime slot 
	 * @return the starttime in timeslots
	 */
	public long getStartTimeSlot(){
		return timeSlot.getStartTimeSlot();
	}
	
	/**
	 * gets the duration of the timeslot
	 * @return the duration in timeslots, can be null!
	 */
	public Long getDuration(){
		return timeSlot.getDuration();
	}
	
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject freeTimeSlotDBObject = new BasicDBObject();
		freeTimeSlotDBObject.put("startTimeSlot", timeSlot.getStartTimeSlot());
		freeTimeSlotDBObject.put("duration", timeSlot.getDuration());
		return freeTimeSlotDBObject;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object)
			throws IllegalArgumentException {
		
		this.timeSlot = new TimeSlot();
		
		BasicDBObject dbObjectCopy= (BasicDBObject) object.copy();
		
		this.timeSlot.setStartTimeSlot(dbObjectCopy.getLong("statTimeSlot"));
		dbObjectCopy.remove("startTimeSlot");
		this.timeSlot.setDuration(dbObjectCopy.getLong("duration"));
		dbObjectCopy.remove("duration");

		if (!dbObjectCopy.isEmpty()){
			throw new IllegalArgumentException("FreeTimeSlot not parsed completely");
		}
		
	}
}