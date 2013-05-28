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

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map.Entry;

import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.log.Logger;
import rexos.mas.data.Part;
import rexos.mas.data.Position;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.StepStatusCode;

import com.mongodb.BasicDBObject;

/**
 * Instances of this class represent the pick & place service and is meant to pick up objects like a cube and put them
 * somewhere else on the working field. It produces 2 service steps: "pick" and "place".
 * 
 * @author Peter Bonnema
 * 
 */
public class PickAndPlaceService extends Service {
	/**
	 * Creates a new PickAndPlaceService object.
	 */
	public PickAndPlaceService() {
		super("huniplacer pick&place", 1);
	}

	/**
	 * @see rexos.mas.service_agent.Service#getServiceSteps(int, com.mongodb.BasicDBObject)
	 */
	@Override
	public ServiceStep[] getServiceSteps(int productStepType, BasicDBObject parameters) {
		Part part = new Part((BasicDBObject) parameters.get("part"));
		double ballHeight = 0;
		try {
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			Row[] partProperties = client.executeSelectQuery(Queries.PART_PROPERTY, part.getType(), "heigth");
			ballHeight = (double) partProperties[0].get("part_type_properties.value");
		} catch(KnowledgeException | KeyNotFoundException e) {
			Logger.log(e);
		}

		BasicDBObject pickParameters = new BasicDBObject();
		pickParameters.put("part", part.toBasicDBObject());
		pickParameters.put("position", new Position().toBasicDBObject());

		Position position = new Position((BasicDBObject) parameters.get("position"));
		position.setZ(position.getZ() + ballHeight);
		BasicDBObject placeParameters = new BasicDBObject();
		placeParameters.put("part", part.toBasicDBObject());
		placeParameters.put("position", position.toBasicDBObject());

		return new ServiceStep[] {
				new ServiceStep(getId(), 1, pickParameters, StepStatusCode.EVALUATING, null, new ScheduleData()),
				new ServiceStep(getId(), 2, placeParameters, StepStatusCode.EVALUATING, null, new ScheduleData())
		};
	}

	/**
	 * @see rexos.mas.service_agent.Service#updateParameters(java.util.HashMap, rexos.mas.service_agent.ServiceStep[])
	 */
	@Override
	public ServiceStep[] updateParameters(HashMap<Part, Position> partParameters, ServiceStep[] serviceSteps) {
		BasicDBObject pickParameters = serviceSteps[0].getParameters();
		Position ballPosition = partParameters.values().toArray(new Position[0])[0];
		pickParameters.put("position", ballPosition.toBasicDBObject());
		serviceSteps[0].setParameters(pickParameters);
		return serviceSteps;
	}
}
