/**
 * @file rexos/mas/service_agent/PickAndPlaceService.java
 * @brief Class for the pick&place service.
 * @date Created: 23 apr. 2013
 * 
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
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

import java.util.AbstractMap.SimpleEntry;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map.Entry;

import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.log.Logger;
import rexos.mas.data.Position;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

public class PickAndPlaceService implements Service {
	private KnowledgeDBClient client;
	private static final int ID = 1;
	private static final String NAME = "huniplacer pick&place";

	/**
	 * @see rexos.mas.service_agent.Service#getModuleIds(int,
	 *      com.mongodb.BasicDBObject)
	 */
	@Override
	public int[] getModuleIds(int productStepType, BasicDBObject parameters) {
		try {
			client = KnowledgeDBClient.getClient();
			Row[] moduleGroups = client.executeSelectQuery(Queries.MODULEGROUPS_REQUIRED_PER_SERVICE, NAME);
			int[] moduleIds = new int[moduleGroups.length];
			for(int i = 0; i < moduleGroups.length; i++) {
				moduleIds[i] = (int) moduleGroups[i].get("module_id");
			}
			return moduleIds;
		} catch(KnowledgeException | KeyNotFoundException e1) {
			Logger.log(e1);
		}
		return null;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getServiceSteps(int,
	 *      com.mongodb.BasicDBObject)
	 */
	@Override
	public ServiceStep[] getServiceSteps(int productStepType, BasicDBObject parameters) {
		int part = parameters.getInt("part");
		double inputPartSize = 0.5;// TODO: FROM KNOWLEDGE DB

		BasicDBObject pickParameters = new BasicDBObject();
		pickParameters.put("part", part);
		pickParameters.put("position", new Position().toBasicDBObject());

		Position position = new Position((BasicDBObject) parameters.get("position"));
		position.setZ(position.getZ() + inputPartSize);
		BasicDBObject placeParameters = new BasicDBObject();
		placeParameters.put("part", part);
		placeParameters.put("position", position.toBasicDBObject());

		return new ServiceStep[] {
				new ServiceStep(ID, 1, pickParameters, StepStatusCode.EVALUATING, null, new ScheduleData()),
				// pick //TODO NOT HARDCODED ID.
				new ServiceStep(ID, 2, placeParameters, StepStatusCode.EVALUATING, null, new ScheduleData())
		// place //TODO NOT HARDCODE ID.
		};
	}

	/* (non-Javadoc)
	 * @see rexos.mas.service_agent.Service#updateParameters(java.util.HashMap,
	 * rexos.mas.service_agent.ServiceStep[]) */
	@Override
	public ServiceStep[] updateParameters(HashMap<Integer, SimpleEntry<Integer, Position>> partParameters,
			ServiceStep[] serviceSteps) {
		BasicDBObject pickParameters = serviceSteps[0].getParameters();
		for(Entry<Integer, SimpleEntry<Integer, Position>> e : partParameters.entrySet()){
			if(e.getValue().getKey() == pickParameters.getInt("part")){
				pickParameters.putAll((LinkedHashMap<String, Object>) 
						new BasicDBObject("position", e.getValue().getValue().toBasicDBObject()));
			}
		}
		serviceSteps[0].setParameters(pickParameters);
		return serviceSteps;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getId()
	 */
	@Override
	public int getId() {
		return ID;
	}

	/**
	 * @see rexos.mas.service_agent.Service#getName()
	 */
	@Override
	public String getName() {
		return NAME;
	}
}
