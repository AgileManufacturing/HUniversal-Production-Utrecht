/**
 * @file rexos/mas/service_agent/ServiceStep.java
 * @brief Provides a message for the servicestep blackboard
 * @date Created: 2013-04-03
 * 
 * @author Peter Bonnema
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 **/
package rexos.mas.service_agent;

import org.bson.types.ObjectId;

import rexos.mas.data.MongoSaveable;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * Represents a service step. It implements the singular linked list principle in it that holds a reference to the next
 * step that should be executed. The reference is in the form of a ObjectId of the next step. If this field is null it
 * means it´s the last step for this product step.
 * 
 * @author Peter
 */
public class ServiceStep implements MongoSaveable {
	private ObjectId _id;
	private ObjectId nextStep;
	private int serviceId;
	private int type;
	private ObjectId productStepId;
	private BasicDBObject parameters;
	private StepStatusCode status;
	private BasicDBObject statusData;
	private ScheduleData scheduleData;

	/**
	 * Creates an empty ServiceStep.
	 */
	public ServiceStep() {}

	/**
	 * Creates a ServiceStep with the specified parameters. The nextStep field and productStepId field will be
	 * initialized to null.
	 * 
	 * @param serviceId The ID of the service that created this ServiceStep.
	 * @param type The type of this service step (e.g. "pick", "place", "move up").
	 * @param parameters The parameters of the step.
	 * @param status The status of the step.
	 * @param statusData The status data describing the status of the step.
	 * @param scheduleData The schedule data containing start time, duration and deadline. The latter is currently
	 *            unused.
	 */
	public ServiceStep(int serviceId, int type, BasicDBObject parameters, StepStatusCode status,
			BasicDBObject statusData, ScheduleData scheduleData) {
		this(null, serviceId, type, parameters, status, statusData, scheduleData);
	}

	/**
	 * Creates a ServiceStep with the specified parameters. The nextStep field will be initialized to null because it
	 * cannot be sensibly filled in before saving this step on the blackboard.
	 * 
	 * @param productStepId The productStep this ServiceStep resulted from.
	 * @param serviceId The ID of the service that created this ServiceStep.
	 * @param type The type of this service step (e.g. "pick", "place", "move up").
	 * @param parameters The parameters of the step.
	 * @param status The status of the step.
	 * @param statusData The status data describing the status of the step.
	 * @param scheduleData The schedule data containing start time, duration and deadline. The latter is currently
	 *            unused.
	 */
	public ServiceStep(ObjectId productStepId, int serviceId, int type, BasicDBObject parameters,
			StepStatusCode status, BasicDBObject statusData, ScheduleData scheduleData) {
		this.nextStep = null;
		this.serviceId = serviceId;
		this.type = type;
		this.productStepId = productStepId;
		this.parameters = parameters;
		this.status = status;
		this.statusData = statusData;
		this.scheduleData = scheduleData;
	}

	/**
	 * Creates a new ServiceStep and initializes all fields with the data in the specified BasicDBObject.
	 * 
	 * @param object The BasicDBObject containing all data to initialized the ServiceStep with.
	 */
	public ServiceStep(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * Sorts the ServiceStepMessages in the specified array bases on their nextStep field. The last step is the one of
	 * which the nextStep field is null.
	 * 
	 * @param unsortedSteps An array of steps to be sorted.
	 * @return An array of ServiceStep in the right order.
	 */
	public static ServiceStep[] sort(ServiceStep[] unsortedSteps) {
		// Find the first step
		ServiceStep firstServiceStep = null;
		outer: for(ServiceStep serviceStep : unsortedSteps) {
			for(ServiceStep serviceStep2 : unsortedSteps) {
				if(serviceStep2.getNextStep() != null && serviceStep2.getNextStep().equals(serviceStep.getId())) {
					continue outer;
				}
			}
			firstServiceStep = serviceStep;
			break;
		}

		// sort all steps beginning with the one found above
		int stepsCount = unsortedSteps.length;
		ObjectId nextStepId;
		ServiceStep[] sortedSteps = new ServiceStep[stepsCount];
		sortedSteps[0] = firstServiceStep;
		for(int i = 1; i < stepsCount; i++) {
			nextStepId = sortedSteps[i - 1].getNextStep();
			for(ServiceStep serviceStep : unsortedSteps) {
				if(serviceStep.getId().equals(nextStepId)) {
					sortedSteps[i] = serviceStep;
					break;
				}
			}
		}
		return sortedSteps;
	}

	/* (non-Javadoc)
	 * @see rexos.mas.data.MongoSaveable#toBasicDBObject() */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject dbObject =
				(BasicDBObject) BasicDBObjectBuilder.start().add("nextStep", nextStep).add("serviceId", serviceId)
						.add("type", type).add("productStepId", productStepId).add("parameters", parameters)
						.add("status", status.name()).add("statusData", statusData)
						.add("scheduleData", scheduleData.toBasicDBObject()).get();

		return dbObject;
	}

	/* (non-Javadoc)
	 * @see rexos.mas.data.MongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject) */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		_id = object.getObjectId("_id");
		nextStep = object.getObjectId("nextStep");
		serviceId = object.getInt("serviceId");
		type = object.getInt("type");
		productStepId = object.getObjectId("productStepId");
		parameters = (BasicDBObject) object.get("parameters");
		status = StepStatusCode.valueOf(object.getString("status"));
		if(object.containsField("statusData")) {
			statusData = (BasicDBObject) object.get(statusData);
		} else {
			statusData = new BasicDBObject();
		}
		if(object.containsField("scheduleData")) {
			scheduleData = new ScheduleData((BasicDBObject) object.get("scheduleData"));
		} else {
			scheduleData = new ScheduleData();
		}
	}

	/**
	 * Returns the MongoDB ObjectId of the entry of this service step on a blackboard if it was recreated from that
	 * entry. Otherwise null is returned.
	 * 
	 * @return The MongoDB ObjectId or null.
	 */
	public ObjectId getId() {
		return _id;
	}

	/**
	 * Returns the ObjectId of the next step to be executed in bunch of steps created by a service object. This follows
	 * the singular linked list pattern.
	 * 
	 * @return The ObjectId of the next step to be executed.
	 */
	public ObjectId getNextStep() {
		return nextStep;
	}

	/**
	 * Sets the ObjectId of the next step to be executed in bunch of steps created by a service object.
	 * 
	 * @param nextStep The ObjectId of the next step to be executed.
	 */
	public void setNextStep(ObjectId nextStep) {
		this.nextStep = nextStep;
	}

	/**
	 * The MongoDB ObjectId of the productStep
	 * 
	 * @return the productStepId
	 */
	public ObjectId getProductStepId() {
		return productStepId;
	}

	/**
	 * @param productStepId
	 *            the productStepId to set
	 */
	public void setProductStepId(ObjectId productStepId) {
		this.productStepId = productStepId;
	}

	/**
	 * @return the type
	 */
	public int getType() {
		return type;
	}

	/**
	 * @param type
	 *            the type to set
	 */
	public void setType(int type) {
		this.type = type;
	}

	/**
	 * @return the serviceId
	 */
	public int getServiceId() {
		return serviceId;
	}

	/**
	 * @param serviceId
	 *            the serviceId to set
	 */
	public void setServiceId(int serviceId) {
		this.serviceId = serviceId;
	}

	/**
	 * @return the parameters
	 */
	public BasicDBObject getParameters() {
		return parameters;
	}

	/**
	 * @param parameters
	 *            the parameters to set
	 */
	public void setParameters(BasicDBObject parameters) {
		this.parameters = parameters;
	}

	/**
	 * @return the status
	 */
	public StepStatusCode getStatus() {
		return status;
	}

	/**
	 * @param status
	 *            the status to set
	 */
	public void setStatus(StepStatusCode status) {
		this.status = status;
	}

	/**
	 * @return the statusData
	 */
	public BasicDBObject getStatusData() {
		return statusData;
	}

	/**
	 * @param statusData
	 *            the statusData to set
	 */
	public void setStatusData(BasicDBObject statusData) {
		this.statusData = statusData;
	}

	/**
	 * @return the scheduleData
	 */
	public ScheduleData getScheduleData() {
		return scheduleData;
	}

	/**
	 * @param scheduleData
	 *            the scheduleData to set
	 */
	public void setScheduleData(ScheduleData scheduleData) {
		this.scheduleData = scheduleData;
	}
}