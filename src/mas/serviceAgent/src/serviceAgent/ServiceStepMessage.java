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
package serviceAgent;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;

import equipletAgent.StepStatusCode;
import newDataClasses.ScheduleData;

/**
 * Implementation of a message for the servicestep blackboard
 */
public class ServiceStepMessage {
	private ObjectId productStepId;
	private String serviceName;
	private long type;
	private BasicDBObject parameters;
	private StepStatusCode status;
	private BasicDBObject statusData;
	private ScheduleData scheduleData;

	public ServiceStepMessage(long type, String serviceName, BasicDBObject parameters,
			StepStatusCode status, BasicDBObject statusData,
			ScheduleData scheduleData) {
		this(null, type, serviceName, parameters, status, statusData, scheduleData);
	}

	public ServiceStepMessage(ObjectId productStepId, long type, String serviceName,
			BasicDBObject parameters, StepStatusCode status,
			BasicDBObject statusData, ScheduleData scheduleData) {
		this.productStepId = productStepId;
		this.type = type;
		this.serviceName = serviceName;
		this.parameters = parameters;
		this.status = status;
		this.statusData = statusData;
		this.scheduleData = scheduleData;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null)
			return false;
		if (obj == this)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		ServiceStepMessage other = (ServiceStepMessage) obj;
		return productStepId.equals(other.productStepId) && type == other.type
				&& serviceName.equals(other.serviceName)
				&& parameters.equals(other.parameters)
				&& status == other.status
				&& statusData.equals(other.statusData)
				&& scheduleData.equals(other.scheduleData);
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		// TODO Auto-generated method stub
		return super.hashCode();
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
	 * @return the serviceName
	 */
	public String getServiceName() {
		return serviceName;
	}

	/**
	 * @param serviceName the serviceName to set
	 */
	public void setServiceName(String serviceName) {
		this.serviceName = serviceName;
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