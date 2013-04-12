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

import java.util.HashMap;

import equipletAgent.StepStatusCode;

import newDataClasses.ScheduleData;

/**
 * Implementation of a message for the servicestep blackboard
 */
public class ServiceStepMessage {
	public long serviceStepId;
	public long productStepId;
	public String type;
	public HashMap<String, String> parameters;
	public StepStatusCode status;
	public HashMap<String, String> statusData;
	public ScheduleData timeData;

	public ServiceStepMessage(int serviceStepId, int productStepId,
							  String type, HashMap<String, String> parameters,
							  StepStatusCode status, HashMap<String, String> statusData, ScheduleData timeData) {
		this.serviceStepId = serviceStepId;
		this.productStepId = productStepId;
		this.type = type;
		this.parameters = parameters;
		this.status = status;
		this.statusData = statusData;
		this.timeData = timeData;
	}
	
	public boolean equals(Object obj) {
		if(obj == null)
			return false;
		if(obj == this)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		ServiceStepMessage other = (ServiceStepMessage) obj;
		return	serviceStepId == other.serviceStepId &&
				productStepId == other.productStepId &&
				type.equals(other.type) &&
				parameters.equals(other.parameters) &&
				status == other.status &&
				statusData.equals(other.statusData) &&
				timeData.equals(other.timeData);
	}
}