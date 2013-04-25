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

import java.util.HashMap;

import com.mongodb.BasicDBObject;
import rexos.libraries.knowledgedb_client.*;
import rexos.mas.data.Position;
import rexos.mas.equiplet_agent.StepStatusCode;

public class PickAndPlaceService implements Service {

	private KnowledgeDBClient client;
	private int id = 0;
	private int saveMovementPlane = 6;

	/**
	 * @see rexos.mas.service_agent.Service#getModuleIds(int, com.mongodb.BasicDBObject)
	 */
	@Override
	public int[] getModuleIds(int productStepType, BasicDBObject parameters) {
		int[] moduleIds = new int[0];
		//ArrayList<Integer> moduleIds = new ArrayList<Integer>();

		try {
			client = KnowledgeDBClient.getClient();

			Object[] queryParameters = { "HUniplacer.pickandplace" };
			Row[] rows = client.executeSelectQuery(Queries.MODULES_REQUIRED_PER_SERVICE, queryParameters);
			moduleIds = new int[rows.length];
			for(int i = 0; i < rows.length; i++){
				moduleIds[i] = (int) rows[i].get("id");
			}
		} catch (KnowledgeException | KeyNotFoundException e1) {
			e1.printStackTrace();
		}
		return new int[] { 1, 2 };// TODO get this working
		//return moduleIds;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getServiceSteps(int, com.mongodb.BasicDBObject)
	 */
	@Override
	public ServiceStepMessage[] getServiceSteps(int productStepType, BasicDBObject parameters) {
		int inputPart = parameters.getInt("InputPart");
		double inputPartSize = 0.5;// TODO: FROM KNOWLEDGE DB
		ServiceStepMessage[] serviceStepMessages = new ServiceStepMessage[2];

		BasicDBObject pickParameters = new BasicDBObject();
		pickParameters.put("InputPart", inputPart);
		pickParameters.put("Position", new Position());
		pickParameters.put("SaveMovementPlane", new BasicDBObject("Height", saveMovementPlane).put("RelativeTo", null));

		Position position = new Position((BasicDBObject) parameters.get("Position"));
		position.setZ(position.getZ() + inputPartSize);
		BasicDBObject placeParameters = new BasicDBObject();
		placeParameters.put("InputPart", inputPart);
		placeParameters.put("Position", position);
		placeParameters.put("SaveMovementPlane", new BasicDBObject("Height", saveMovementPlane).put("RelativeTo", inputPart));

		serviceStepMessages[0] = new ServiceStepMessage(id, 4, pickParameters, StepStatusCode.EVALUATING, null, null);// pick //TODO: NOT HARDCODED ID.
		serviceStepMessages[1] = new ServiceStepMessage(id, 5, placeParameters, StepStatusCode.EVALUATING, null, null);// place //TODOD NOT HARDCODE ID.
		return serviceStepMessages;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getId()
	 */
	@Override
	public int getId() {
		return id;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getName()
	 */
	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	/* (non-Javadoc)
	 * @see rexos.mas.service_agent.Service#updateParameters(java.util.HashMap, rexos.mas.service_agent.ServiceStepMessage[])
	 */
	@Override
	public ServiceStepMessage[] updateParameters(
			HashMap<Long, Position> partParameters,
			ServiceStepMessage[] serviceSteps) {
		// TODO Auto-generated method stub
		return null;
	}
}
