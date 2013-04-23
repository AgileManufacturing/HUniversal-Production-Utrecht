/**
 * @file EquipletStepMessage.java
 * @brief Makes a messages to be used for a 
 * @brief respond. This respond consist of a serviceStepID,instructiondata a type and timedata.
 * @date Created: 2013-04-02
 *
 * @author Thierry Gerritse
 * @author Wouter Veen
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
package rexos.mas.hardware_agent;

import jade.core.AID;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;

import rexos.mas.data.IMongoSaveable;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.StepStatusCode;

public class EquipletStepMessage implements IMongoSaveable {
	private ObjectId serviceStepID;
	private Long instructionData;
	private Long type;
	private TimeData timeData;
	
	/**
	 * @param serviceStepID
	 * @param ionstructionData
	 * @param type
	 * @param timeData
	 * @return
	 */
	public EquipletStepMessage(ObjectId serviceStepID,
							  Long instructionData, Long type, TimeData timeData){
		
		this.serviceStepID = serviceStepID;
		this.instructionData = instructionData;
		this.timeData = timeData;
		this.type = type;
	}

	/**
	 * @return the serviceStepID
	 */
	public ObjectId getServiceStepID() {
		return serviceStepID;
	}

	/**
	 * @param serviceStepID the serviceStepID to set
	 */
	public void setServiceStepID(ObjectId serviceStepID) {
		this.serviceStepID = serviceStepID;
	}

	/**
	 * @return the instructionData
	 */
	public Long getInstructionData() {
		return instructionData;
	}

	/**
	 * @param instructionData the instructionData to set
	 */
	public void setInstructionData(Long instructionData) {
		this.instructionData = instructionData;
	}

	/**
	 * @return the type
	 */
	public Long getType() {
		return type;
	}

	/**
	 * @param type the type to set
	 */
	public void setType(Long type) {
		this.type = type;
	}

	/**
	 * @return the timeData
	 */
	public TimeData getTimeData() {
		return timeData;
	}

	/**
	 * @param timeData the timeData to set
	 */
	public void setTimeData(TimeData timeData) {
		this.timeData = timeData;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String
				.format("EquipletStepMessage [serviceStepID=%s, instructionData=%s, type=%s, timeData=%s]",
						serviceStepID, instructionData, type, timeData);
	}

	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("serviceStepID", serviceStepID);
		object.put("instructionData", instructionData);
		object.put("type", type);
		object.put("timeData", timeData);
		return object;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		serviceStepID = object.getObjectId("serviceStepID");
		instructionData = object.getLong("instructionData");
		type = object.getLong("type");
		timeData = new TimeData(object.getLong("timeData"));	
	}
}
