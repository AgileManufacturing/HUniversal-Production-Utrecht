/**
 * @file ServiceStepMessage.java
 * @brief Provides a message for the servicestep blackboard
 * @date Created: 2013-04-03
 *
 * @author Hessel Meulenbeld
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
 **/
package rexos.mas.service_agent;

import org.bson.types.ObjectId;

import rexos.mas.data.IMongoSaveable;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * Implementation of a message for the serviceStep blackboard
 * 
 * @author Peter
 */
public class ServiceStepMessage implements IMongoSaveable {
	private ObjectId _id;
	private long serviceId;
	private long type;
	private ObjectId productStepId;
	private BasicDBObject parameters;
	private StepStatusCode status;
	private BasicDBObject statusData;
	private ScheduleData scheduleData;

	/**
	 * @param serviceId
	 * @param type
	 * @param parameters
	 * @param status
	 * @param statusData
	 * @param scheduleData
	 */
	public ServiceStepMessage(long serviceId, long type,
			BasicDBObject parameters, StepStatusCode status,
			BasicDBObject statusData, ScheduleData scheduleData) {
		this(null, serviceId, type, null, parameters, status,
				statusData, scheduleData);
	}

	/**
	 * @param serviceId
	 * @param type
	 * @param productStepId
	 * @param parameters
	 * @param status
	 * @param statusData
	 * @param scheduleData
	 */
	public ServiceStepMessage(long serviceId,
			long type, ObjectId productStepId, BasicDBObject parameters, StepStatusCode status,
			BasicDBObject statusData, ScheduleData scheduleData) {
		this(null, serviceId, type, productStepId, parameters, status,
				statusData, scheduleData);
	}

	/**
	 * @param _id
	 * @param serviceId
	 * @param type
	 * @param parameters
	 * @param status
	 * @param statusData
	 * @param scheduleData
	 */
	public ServiceStepMessage(ObjectId _id, long serviceId, long type,
			BasicDBObject parameters, StepStatusCode status,
			BasicDBObject statusData, ScheduleData scheduleData) {
		this(_id, serviceId, type, null, parameters, status,
				statusData, scheduleData);
	}

	/**
	 * @param _id
	 * @param serviceId
	 * @param type
	 * @param productStepId
	 * @param parameters
	 * @param status
	 * @param statusData
	 * @param scheduleData
	 */
	public ServiceStepMessage(ObjectId _id, long serviceId, long type, ObjectId productStepId, BasicDBObject parameters,
			StepStatusCode status, BasicDBObject statusData,
			ScheduleData scheduleData) {
		this._id = _id;
		this.productStepId = productStepId;
		this.serviceId = serviceId;
		this.type = type;
		this.parameters = parameters;
		this.status = status;
		this.statusData = statusData;
		this.scheduleData = scheduleData;
	}

	/**
	 * @param object
	 */
	public ServiceStepMessage(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * @return
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject dbObject = (BasicDBObject) BasicDBObjectBuilder.start()
				.add("productStepId", productStepId)
				.add("serviceId", serviceId).add("type", type)
				.add("parameters", parameters).add("status", status.name())
				.add("statusData", statusData)
				.add("scheduleData", scheduleData.toBasicDBObject()).get();

		if (_id != null)
			dbObject.append("_id", _id);

		return dbObject;
	}

	/**
	 * @param object
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		_id = object.getObjectId("_id");
		productStepId = object.getObjectId("productStepId");
		serviceId = object.getLong("serviceId");
		type = object.getLong("type");
		parameters = (BasicDBObject) object.get("parameters");
		status = StepStatusCode.valueOf(object.getString("status"));
		if (object.containsField("statusData")) {
			statusData = (BasicDBObject) object.get(statusData);
		} else {
			statusData = new BasicDBObject();
		}
		if (object.containsField("scheduleData")) {
			scheduleData = new ScheduleData(
					(BasicDBObject) object.get("scheduleData"));
		} else {
			scheduleData = new ScheduleData();
		}
	}

	/**
	 * @return the id
	 */
	public ObjectId getId() {
		return _id;
	}

	/**
	 * @param id
	 *            the id to set
	 */
	public void setId(ObjectId id) {
		_id = id;
	}

	/**
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
	public long getType() {
		return type;
	}

	/**
	 * @param type
	 *            the type to set
	 */
	public void setType(long type) {
		this.type = type;
	}

	/**
	 * @return the serviceId
	 */
	public long getServiceId() {
		return serviceId;
	}

	/**
	 * @param serviceId
	 *            the serviceId to set
	 */
	public void setServiceId(long serviceId) {
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