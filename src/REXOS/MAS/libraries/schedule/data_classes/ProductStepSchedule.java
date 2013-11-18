/**
 * @file REXOS/MAS/libraries/schedule/data_classes/ProductStepSchedule.java
 * @brief Data object for the schedule of a product step primarily in the planningblackboard
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

import java.io.Serializable;

import libraries.blackboard_client.data_classes.MongoSaveable;
import com.mongodb.BasicDBObject;

import org.bson.types.ObjectId;


/**
 * Data class representing a product step scheduled in  the scheduleboard.
 * implements mongosavable for saving in blackboards.
 * @author roy
 *
 */
public class ProductStepSchedule implements MongoSaveable, Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 5464110443463847485L;

	/**
	 * @var ObjectId objectId
	 * 		the objectId of the productStepId
	 */
	private ObjectId objectId;
	
	/**
	 * @var long startTime
	 * 		The start time in timeslots.
	 **/
	private long startTime;
	
	/**
	 * @var long duration
	 * The duration of the step in timeslots.
	 **/
	private long duration;
	
	/**
	 * @var long deadline
	 * The deadline in timeslots.
	 **/
	private long deadline;
	
	/**
	 * Standard constructor
	 * @param objectid the objectId of the productStep to be scheduled
	 * @param startTime the starttime of the scheduled step. In timeslots
	 * @param duration the duration of this productStep. In timeslots
	 * @param deadline the deadline of this productStep. In timeslots
	 */
	public ProductStepSchedule(ObjectId objectid, long startTime, long duration, long deadline){
		this.objectId = objectid;
		this.startTime = startTime;
		this.duration = duration;
		this.deadline = deadline;
	}
	
	/**
	 * Constructor witch sets the timedata to -1
	 * @param objectId the objectId of the productstep
	 */
	public ProductStepSchedule(ObjectId objectId){
		this(objectId, -1, -1, -1);
	}
	
	/**
	 * constructor for converting a mongo DBObject to a ProductStepSchedule
	 * @param basicDBObject the mongo DBObject
	 */
	public ProductStepSchedule(BasicDBObject basicDBObject){
		fromBasicDBObject(basicDBObject);
	}
	
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject productStepScheduleDBObject = new BasicDBObject();
		productStepScheduleDBObject.put("objectId", objectId);
		productStepScheduleDBObject.put("startTime", startTime);
		productStepScheduleDBObject.put("duration", duration);
		productStepScheduleDBObject.put("deadline", deadline);
		return productStepScheduleDBObject;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object)
			throws IllegalArgumentException {
		if (object != null){
			BasicDBObject dBObjectCopy= (BasicDBObject) object.copy();
			
			this.objectId = (ObjectId) dBObjectCopy.remove("objectId");
			
			this.startTime = dBObjectCopy.getLong("startTime", -1);
			dBObjectCopy.remove("startTime");
			this.duration = dBObjectCopy.getLong("duration", -1);
			dBObjectCopy.remove("duration");
			this.deadline = dBObjectCopy.getLong("deadline", -1);
			dBObjectCopy.remove("deadline");
			
			if (!dBObjectCopy.isEmpty()){
				throw new IllegalArgumentException("BasicDBObject not parsed completely");
			}
		}
	}

}
