/**
 * @file rexos/mas/service_agent/ServiceStep.java
 * @brief Provides a message for the serviceStep blackboard.
 * @date Created: 2013-04-03
 * 
 * @author Peter Bonnema
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2013, HU University of Applied Sciences Utrecht.
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
 * @author Peter Bonnema
 */
public class ServiceStep implements MongoSaveable {
	/**
	 * @var ObjectId _id
	 *      The MongoDb ObjectId of this serviceStep.
	 */
	private ObjectId _id;

	/**
	 * @var ObjectId nextStep
	 *      The ObjectId of the next serviceStep to be executed.
	 */
	private ObjectId nextStep;

	/**
	 * @var int serviceId
	 *      The id of the service that made this serviceStep.
	 */
	private int serviceId;

	/**
	 * @var int type
	 *      The type of this serviceStep. It stands for things like "Pick" or "Place".
	 */
	private int type;

	/**
	 * @var ObjectId productStepId
	 *      The MongoDB ObjectId of the productStep this serviceStep belongs to.
	 */
	private ObjectId productStepId;

	/**
	 * @var BasicDBObject parameters
	 *      The parameters.
	 */
	private BasicDBObject parameters;

	/**
	 * @var StepStatusCode status
	 *      The status of this step.
	 */
	private StepStatusCode status;

	/**
	 * @var BasicDBObject statusData
	 *      The statusData giving extra information like reason/source in case of ABORTED.
	 */
	private BasicDBObject statusData;

	/**
	 * @var ScheduleData scheduleData
	 *      The scheduleData containing startTime, duration and deadline.
	 */
	private ScheduleData scheduleData;

	/**
	 * Creates an empty ServiceStep.
	 */
	public ServiceStep() {}

	/**
	 * Creates a ServiceStep with the specified parameters. The nextStep field and productStepId field will be
	 * initialized to null.
	 * 
	 * @param serviceId The Knowledge database id of the service that created this ServiceStep.
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
	 * @param productStepId The ObjectId of the productStep this ServiceStep resulted from.
	 * @param serviceId The Knowledge database id of the service that created this ServiceStep.
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
	 * @param unsortedSteps an array of steps to be sorted.
	 * @return an array of ServiceStep in the right order.
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

	/**
	 * @see rexos.mas.data.MongoSaveable#toBasicDBObject()
	 */
	//@formatter:off
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject dbObject =
				(BasicDBObject) BasicDBObjectBuilder.start()
					.add("nextStep", nextStep)
					.add("serviceId", serviceId)
					.add("type", type)
					.add("productStepId", productStepId)
					.add("parameters", parameters)
					.add("status", status.name())
					.add("statusData", statusData)
					.add("scheduleData", scheduleData.toBasicDBObject()).get();

		return dbObject;
	} //@formatter:on

	/**
	 * @see rexos.mas.data.MongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		_id = (ObjectId) object.remove("_id");
		nextStep = (ObjectId) object.remove("nextStep");
		serviceId = (int) object.remove("serviceId");
		type = (int) object.remove("type");
		productStepId = (ObjectId) object.remove("productStepId");
		parameters = (BasicDBObject) object.remove("parameters");
		status = StepStatusCode.valueOf((String) object.remove("status"));
		if(object.containsField("statusData")) {
			statusData = (BasicDBObject) object.remove("statusData");
		} else {
			statusData = new BasicDBObject();
		}
		if(object.containsField("scheduleData")) {
			scheduleData = new ScheduleData((BasicDBObject) object.remove("scheduleData"));
		} else {
			scheduleData = new ScheduleData();
		}
		if(!object.isEmpty()){
			throw new IllegalArgumentException();
		}
	}

	/**
	 * Returns the MongoDB ObjectId of the entry of this service step on a blackboard if it was recreated from that
	 * entry. Otherwise null is returned.
	 * 
	 * @return the MongoDB ObjectId or null.
	 */
	public ObjectId getId() {
		return _id;
	}

	/**
	 * Returns the ObjectId of the next step to be executed in a sequence of steps created by a service object. This
	 * follows the singular linked list pattern.
	 * 
	 * @return the ObjectId of the next step to be executed.
	 */
	public ObjectId getNextStep() {
		return nextStep;
	}

	/**
	 * Sets the ObjectId of the next step to be executed. Use this to setup the linked list as a sequence of steps to be
	 * executed one after another.
	 * 
	 * @param nextStep the ObjectId of the next step to be executed.
	 */
	public void setNextStep(ObjectId nextStep) {
		this.nextStep = nextStep;
	}

	/**
	 * Returns the MongoDB ObjectId of the productStep from which this service step resulted from.
	 * 
	 * @return the MongoDB ObjectId of the productStep from which this service step resulted from.
	 */
	public ObjectId getProductStepId() {
		return productStepId;
	}

	/**
	 * Sets the MongoDB ObjectId of the productStep from which this service step resulted from.
	 * 
	 * @param productStepId the MongoDB ObjectId of the productStep from which this service step resulted from.
	 */
	public void setProductStepId(ObjectId productStepId) {
		this.productStepId = productStepId;
	}

	/**
	 * Returns the type of this step.
	 * 
	 * @return the type of this step.
	 */
	public int getType() {
		return type;
	}

	/**
	 * Sets the type of this step.
	 * 
	 * @param type the type of this step.
	 */
	public void setType(int type) {
		this.type = type;
	}

	/**
	 * Returns the Knowledge database id of the service.
	 * 
	 * @return the Knowledge database id of the service.
	 */
	public int getServiceId() {
		return serviceId;
	}

	/**
	 * Sets the Knowledge database id of the service.
	 * 
	 * @param serviceId the Knowledge database id of the service.
	 */
	public void setServiceId(int serviceId) {
		this.serviceId = serviceId;
	}

	/**
	 * Returns the parameters.
	 * 
	 * @return the parameters
	 */
	public BasicDBObject getParameters() {
		return parameters;
	}

	/**
	 * Sets the parameters.
	 * 
	 * @param parameters the parameters to set
	 */
	public void setParameters(BasicDBObject parameters) {
		this.parameters = parameters;
	}

	/**
	 * Returns the status.
	 * 
	 * @return the status
	 */
	public StepStatusCode getStatus() {
		return status;
	}

	/**
	 * Sets the status.
	 * 
	 * @param status the status to set
	 */
	public void setStatus(StepStatusCode status) {
		this.status = status;
	}

	/**
	 * Returns the status data.
	 * 
	 * @return the statusData
	 */
	public BasicDBObject getStatusData() {
		return statusData;
	}

	/**
	 * Sets the status data.
	 * 
	 * @param statusData the statusData to set
	 */
	public void setStatusData(BasicDBObject statusData) {
		this.statusData = statusData;
	}

	/**
	 * Returns the schedule data.
	 * 
	 * @return the scheduleData
	 */
	public ScheduleData getScheduleData() {
		return scheduleData;
	}

	/**
	 * Sets the schedule data.
	 * 
	 * @param scheduleData the scheduleData to set
	 */
	public void setScheduleData(ScheduleData scheduleData) {
		this.scheduleData = scheduleData;
	}
}