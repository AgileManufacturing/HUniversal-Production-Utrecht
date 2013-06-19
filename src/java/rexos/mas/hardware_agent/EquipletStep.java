/**
 * @file rexos/mas/hardware_agent/EquipletStep.java
 * @brief Makes a messages to be used for a
 * @brief respond. This respond consist of a serviceStepID,instructiondata a type and timedata.
 * @date Created: 2013-04-02
 * 
 * @author Thierry Gerritse
 * @author Wouter Veen
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package rexos.mas.hardware_agent;

import org.bson.types.ObjectId;

import rexos.mas.data.MongoSaveable;
import rexos.mas.equiplet_agent.StepStatusCode;
import rexos.mas.hardware_agent.EquipletStep;

import com.mongodb.BasicDBObject;

/**
 * EquipletStep class containing all the data for a message on the
 * EquipletStepBlackboard.
 */
public class EquipletStep implements MongoSaveable {
	/**
	 * @var ObjectId _id
	 *      The id of this step.
	 */
	private ObjectId _id;
	/**
	 * @var ObjectId serviceStepID
	 *      The id of the service step related to this step.
	 */
	private ObjectId serviceStepID;
	/**
	 * @var ObjectId nextStep
	 *      The id of the next step for the same service.
	 */
	private ObjectId nextStep;
	/**
	 * @var int moduleId
	 *      The id of the module that created this step.
	 */
	private int moduleId;
	/**
	 * @var InstructionData instructionData
	 *      The instruction data for this step.
	 */
	private InstructionData instructionData;
	/**
	 * @var StepStatusCode status
	 *      The status of this step.
	 */
	private StepStatusCode status;

	/**
	 * @var BasicDBObject statusData
	 *      The extra data provided by the status for
	 *      this product step.
	 */
	private BasicDBObject statusData;

	/**
	 * @var TimeData timeData
	 *      The time data of this step.
	 */
	private TimeData timeData;

	/**
	 * Constructor for this class.
	 * 
	 * @param serviceStepID
	 *            The serviceStepId
	 * @param moduleId
	 *            The moduleId
	 * @param instructionData
	 *            The instructionData
	 * @param status
	 *            The status
	 * @param statusData
	 *            The extra data by the status
	 * @param timeData
	 *            The timeData
	 */
	public EquipletStep(ObjectId serviceStepID, int moduleId, InstructionData instructionData, StepStatusCode status,
			BasicDBObject statusData, TimeData timeData) {

		this.serviceStepID = serviceStepID;
		this.moduleId = moduleId;
		this.instructionData = instructionData;
		this.status = status;
		this.statusData = statusData;
		this.timeData = timeData;
	}

	/**
	 * Constructor building this class from a BasicDBObject
	 * 
	 * @param object
	 *            The BasicDBObject to build this class from.
	 */
	public EquipletStep(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * Getter for the id.
	 * 
	 * @return the id
	 */
	public ObjectId getId() {
		return _id;
	}

	/**
	 * Getter for the serviceStepID.
	 * 
	 * @return the serviceStepID
	 */
	public ObjectId getServiceStepID() {
		return serviceStepID;
	}

	/**
	 * Setter for the ServiceStepID
	 * 
	 * @param serviceStepID
	 *            The serviceStepID to set it to.
	 */
	public void setServiceStepID(ObjectId serviceStepID) {
		this.serviceStepID = serviceStepID;
	}

	/**
	 * Setter for the nextStep
	 * 
	 * @param EquipletStep
	 *            The EquipletStep to set it to.
	 */
	public void setNextStep(ObjectId EquipletStep) {
		this.nextStep = EquipletStep;
	}

	/**
	 * Getter for the nextStep
	 * 
	 * @return The nextStep
	 */
	public ObjectId getNextStep() {
		return nextStep;
	}

	/**
	 * Getter for the module id
	 * 
	 * @return The moduleId
	 */
	public int getModuleId() {
		return moduleId;
	}

	/**
	 * Setter for the moduleId
	 * 
	 * @param moduleId
	 *            The moduleId to set it to.
	 */
	public void setModuleId(int moduleId) {
		this.moduleId = moduleId;
	}

	/**
	 * Getter for the instruction data.
	 * 
	 * @return the instructionData
	 */
	public InstructionData getInstructionData() {
		return instructionData;
	}

	/**
	 * Setter for the instruction data.
	 * 
	 * @param instructionData
	 *            The instructionData to set it to
	 */
	public void setInstructionData(InstructionData instructionData) {
		this.instructionData = instructionData;
	}

	/**
	 * Getter for the status
	 * 
	 * @return the status
	 */
	public StepStatusCode getStatus() {
		return status;
	}

	/**
	 * Setter for the status
	 * 
	 * @param status
	 *            The status to set it to
	 */
	public void setStatus(StepStatusCode status) {
		this.status = status;
	}

	/**
	 * Getter for the statusData
	 * 
	 * @return the statusData
	 */
	public BasicDBObject getStatusData() {
		return statusData;
	}

	/**
	 * Setter for the statusData
	 * 
	 * @param statusData
	 *            the statusData to set it to
	 */
	public void setStatusData(BasicDBObject statusData) {
		this.statusData = statusData;
	}

	/**
	 * Getter for the time data
	 * 
	 * @return the timeData
	 */
	public TimeData getTimeData() {
		return timeData;
	}

	/**
	 * Setter for the time data
	 * 
	 * @param timeData
	 *            The timeData to set it to
	 */
	public void setTimeData(TimeData timeData) {
		this.timeData = timeData;
	}

	/**
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String.format("EquipletStep [serviceStepID=%s, instructionData=%s, status=%s, timeData=%s]",
				serviceStepID, instructionData, status, timeData);
	}

	/**
	 * @see rexos.mas.data.MongoSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("serviceStepID", serviceStepID);
		object.put("nextStep", nextStep);
		object.put("moduleId", moduleId);
		object.put("instructionData", instructionData.toBasicDBObject());
		object.put("status", status.toString());
		object.put("statusData", statusData);
		object.put("timeData", timeData.toBasicDBObject());
		return object;
	}

	/**
	 * @see rexos.mas.data.MongoSaveable#fromBasicDBObject(BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		BasicDBObject copy = (BasicDBObject) object.copy();
		_id = (ObjectId) copy.remove("_id");
		nextStep = (ObjectId) copy.remove("nextStep");
		serviceStepID = (ObjectId) copy.remove("serviceStepID");
		moduleId = (int) copy.remove("moduleId");
		instructionData = new InstructionData((BasicDBObject) copy.remove("instructionData"));
		status = StepStatusCode.valueOf((String) copy.remove("status"));
		if(copy.containsField("statusData")) {
			statusData = (BasicDBObject) copy.remove("statusData");
		} else {
			statusData = new BasicDBObject();
		}
		timeData = new TimeData((BasicDBObject) copy.remove("timeData"));
		if(!copy.isEmpty()){
			throw new IllegalArgumentException();
		}
	}

	/**
	 * Sorts the EquipletStepMessage in the specified array bases on their nextStep field. The last step is the one of
	 * which the nextStep field is null.
	 * 
	 * @param unsortedSteps an array of steps to be sorted.
	 * @return an array of EquipletStep in the right order.
	 */
	public static EquipletStep[] sort(EquipletStep[] unsortedSteps) {
		// Find the first step
		EquipletStep firstServiceStep = null;
		outer: for(EquipletStep serviceStep : unsortedSteps) {
			for(EquipletStep equipletStep2 : unsortedSteps) {
				if(equipletStep2.getNextStep() != null && equipletStep2.getNextStep().equals(serviceStep.getId())) {
					continue outer;
				}
			}
			firstServiceStep = serviceStep;
			break;
		}

		// sort all steps beginning with the one found above
		int stepsCount = unsortedSteps.length;
		ObjectId nextStepId;
		EquipletStep[] sortedSteps = new EquipletStep[stepsCount];
		sortedSteps[0] = firstServiceStep;
		for(int i = 1; i < stepsCount; i++) {
			nextStepId = sortedSteps[i - 1].getNextStep();
			for(EquipletStep serviceStep : unsortedSteps) {
				if(serviceStep.getId().equals(nextStepId)) {
					sortedSteps[i] = serviceStep;
					break;
				}
			}
		}
		return sortedSteps;
	}
}
