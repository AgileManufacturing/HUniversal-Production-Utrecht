/**
 * @file rexos/mas/service_agent/DrawLineService.java
 * @brief 
 * @date Created: 18 jun. 2013
 *
 * @author Peter Bonnema
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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
 * 
 **/
package agents.service_agent;

import java.util.HashMap;

import agents.data_classes.Part;
import agents.data_classes.Position;
import agents.data_classes.ScheduleData;
import agents.data_classes.StepStatusCode;

import com.mongodb.BasicDBObject;

/**
 * @author Peter Bonnema
 *
 */
public class DrawLineService extends Service {
	/**
	 * @see rexos.mas.service_agent.Service#canDoStep(int, com.mongodb.BasicDBObject)
	 */
	@SuppressWarnings("unused")
	@Override
	public boolean canDoStep(int productStepType, BasicDBObject parameters) {
		try {
			if(parameters.containsField("startPosition") && parameters.containsField("endPosition")) {
				new Position((BasicDBObject) parameters.get("startPosition"));
				new Position((BasicDBObject) parameters.get("endPosition"));
			} else {
				return false;
			}
		} catch (IllegalArgumentException e) {
			return false;
		}
		return true;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getServiceSteps(int, com.mongodb.BasicDBObject)
	 */
	@Override
	public ServiceStep[] getServiceSteps(int productStepType, BasicDBObject parameters) 
	{
		Position from = new Position((BasicDBObject) parameters.get("startPosition"));
		Position to = new Position((BasicDBObject) parameters.get("endPosition"));

		BasicDBObject serviceStepParameters = new BasicDBObject();
		serviceStepParameters.put("startPosition", from.toBasicDBObject());
		serviceStepParameters.put("endPosition", to.toBasicDBObject());

		return new ServiceStep[]
		{
				new ServiceStep(getId(), 3, serviceStepParameters, StepStatusCode.EVALUATING, null, new ScheduleData())
		};
	}

	/**
	 * @see rexos.mas.service_agent.Service#updateParameters(java.util.HashMap, rexos.mas.service_agent.ServiceStep[])
	 */
	@Override
	public ServiceStep[] updateParameters(HashMap<Part, Position> partParameters, ServiceStep[] serviceSteps) {
		return serviceSteps;
	}
}
