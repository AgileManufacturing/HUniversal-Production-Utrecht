/**
 * @file PickAndPlaceService.java
 * @brief Class for the pick&place service.
 * @date Created: 23 apr. 2013
 *
 * @author Hessel Meulenbeld
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
package rexos.mas.service_agent;

import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;

import com.mongodb.BasicDBObject;
import rexos.libraries.knowledgedb_client.*;
import rexos.mas.equiplet_agent.StepStatusCode;


public class PickAndPlaceService implements Service {

	private KnowledgeDBClient client;
	private long id = 0l;
	
	/**
	 * @see rexos.mas.service_agent.Service#getModuleIds(long, com.mongodb.BasicDBObject)
	 */
	@Override
	public long[] getModuleIds(long productStepType, BasicDBObject parameters) {
		ArrayList<Long> moduleIds = new ArrayList<Long>();
		
		client = KnowledgeDBClient.getClient();
		try {
			Object[] queryParameters = {"HUniplacer.pickandplace"};
			ResultSet resultSet = client.executeSelectQuery(Queries.MODULES_REQUIRED_PER_SERVICE, queryParameters);
			while(resultSet.next()){
				Row row = new Row(resultSet);
				moduleIds.add((long)row.get("id"));
			}
			resultSet.close();
		} catch (SQLException e) {
			e.printStackTrace();
		}
		long[] moduleIdsArray = new long[moduleIds.size()];
		for(int i = 0; i < moduleIds.size(); i++){
			moduleIdsArray[i] = moduleIds.get(i);
		}
		return moduleIdsArray;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getServiceSteps(long, com.mongodb.BasicDBObject)
	 */
	@Override
	public ServiceStepMessage[] getServiceSteps(long productStepType, BasicDBObject parameters) {
		ServiceStepMessage[] serviceStepMessages = new ServiceStepMessage[2];
		serviceStepMessages[0] = new ServiceStepMessage(id, 4l, parameters, StepStatusCode.EVALUATING , null, null);//pick
		serviceStepMessages[1] = new ServiceStepMessage(id, 5l, parameters, StepStatusCode.EVALUATING, null, null);//place
		return serviceStepMessages;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getId()
	 */
	@Override
	public long getId() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getName()
	 */
	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

}
