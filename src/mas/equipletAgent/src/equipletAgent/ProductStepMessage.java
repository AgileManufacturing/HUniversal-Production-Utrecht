/**
 * @file ProductStepMessage.java
 * @brief Provides a message for the productstep blackboard
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
package equipletAgent;

import java.util.HashMap;

import jade.core.AID;

import newDataClasses.ParameterList;
import newDataClasses.ScheduleData;

/**
 * Implementation of a message for the productstep blackboard
 */
public class ProductStepMessage {
	private AID productAgentId;
	private long type;
	private ParameterList parameters;
	private Object inputParts;
	private Object outputParts;
	private StepStatusCode status;
	private HashMap<String, String> statusData;
	private ScheduleData scheduleData;

	public ProductStepMessage(AID productAgentId, long type,
			ParameterList parameters, Object inputParts, Object outputParts,
			StepStatusCode status, HashMap<String, String> statusData, ScheduleData scheduleData) {
		this.productAgentId = productAgentId;
		this.type = type;
		this.parameters = parameters;
		this.inputParts = inputParts;
		this.outputParts = outputParts;
		this.status = status;
		this.statusData = statusData;
		this.scheduleData = scheduleData;
	}

	public boolean equals(Object obj) {
		if (obj == null)
			return false;
		if (obj == this)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		ProductStepMessage other = (ProductStepMessage) obj;
		return productAgentId.equals(other.productAgentId)
				&& type == other.type
				&& parameters.equals(other.parameters)
				&& inputParts.equals(other.inputParts)
				&& outputParts.equals(other.outputParts)
				&& status == other.status
				&& statusData.equals(other.statusData)
				&& scheduleData.equals(other.scheduleData);
	}

	/**
	 * @return the productAgentId
	 */
	public AID getProductAgentId() {
		return productAgentId;
	}

	/**
	 * @param productAgentId the productAgentId to set
	 */
	public void setProductAgentId(AID productAgentId) {
		this.productAgentId = productAgentId;
	}

	/**
	 * @return the type
	 */
	public long getType() {
		return type;
	}

	/**
	 * @param type the type to set
	 */
	public void setType(long type) {
		this.type = type;
	}

	/**
	 * @return the parameters
	 */
	public ParameterList getParameters() {
		return parameters;
	}

	/**
	 * @param parameters the parameters to set
	 */
	public void setParameters(ParameterList parameters) {
		this.parameters = parameters;
	}

	/**
	 * @return the inputParts
	 */
	public Object getInputParts() {
		return inputParts;
	}

	/**
	 * @param inputParts the inputParts to set
	 */
	public void setInputParts(Object inputParts) {
		this.inputParts = inputParts;
	}

	/**
	 * @return the outputParts
	 */
	public Object getOutputParts() {
		return outputParts;
	}

	/**
	 * @param outputParts the outputParts to set
	 */
	public void setOutputParts(Object outputParts) {
		this.outputParts = outputParts;
	}

	/**
	 * @return the status
	 */
	public StepStatusCode getStatus() {
		return status;
	}

	/**
	 * @param status the status to set
	 */
	public void setStatus(StepStatusCode status) {
		this.status = status;
	}

	/**
	 * @return the statusData
	 */
	public HashMap<String, String> getStatusData() {
		return statusData;
	}

	/**
	 * @param statusData the statusData to set
	 */
	public void setStatusData(HashMap<String, String> statusData) {
		this.statusData = statusData;
	}

	/**
	 * @return the scheduleData
	 */
	public ScheduleData getScheduleData() {
		return scheduleData;
	}

	/**
	 * @param scheduleData the scheduleData to set
	 */
	public void setScheduleData(ScheduleData scheduleData) {
		this.scheduleData = scheduleData;
	}
}