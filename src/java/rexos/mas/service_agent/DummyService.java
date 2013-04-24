/**
 * @file DummyService.java
 * @brief 
 * @date Created: 12 apr. 2013
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
package rexos.mas.service_agent;

//import java.util.HashMap;

//import rexos.mas.data.ParameterList;
//import rexos.mas.data.Position;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

/**
 * Just a dummy service for testing purposes.
 * 
 * @author Peter
 * 
 */
public class DummyService implements Service {
	private static final long id = 1l;
	private static final String name = "DummyService";

	/**
	 * Creates a new DummyService.
	 */
	public DummyService() {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.service_agent.Service#getModuleIds(long,
	 * com.mongodb.BasicDBObject)
	 */
	@Override
	public long[] getModuleIds(long productStepType, BasicDBObject parameters) {
		return new long[] { 1l, 2l };
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.service_agent.Service#getServiceSteps(long,
	 * com.mongodb.BasicDBObject)
	 */
	@Override
	public ServiceStepMessage[] getServiceSteps(long productStepType,
			BasicDBObject parameters) {
		ServiceStepMessage service = new ServiceStepMessage(1l, 1, parameters,
				StepStatusCode.EVALUATING, new BasicDBObject("status",
						"dummy status"), new ScheduleData());
		return new ServiceStepMessage[] { service };
	}

//	/* (non-Javadoc)
//	 * @see rexos.mas.service_agent.Service#getParameters(java.util.HashMap)
//	 */
//	@Override
//	public ParameterList getParameters(HashMap<Long, Position> partParameters) {
//		return null;
//	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.service_agent.Service#getId()
	 */
	@Override
	public long getId() {
		return id;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.service_agent.Service#getName()
	 */
	@Override
	public String getName() {
		return name;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String.format("DummyService [getId()=%s, getName()=%s]",
				getId(), getName());
	}
}
