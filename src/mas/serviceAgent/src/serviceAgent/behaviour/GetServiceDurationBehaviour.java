/**
 * @file GetServiceStepBehaviour.java
 * @brief 
 * @date Created: 11 apr. 2013
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
package serviceAgent.behaviour;

import java.io.IOException;

import org.bson.types.ObjectId;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.mongodb.DBObject;

import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;
import nl.hu.client.InvalidJSONException;
import equipletAgent.ServiceStepMessage;
import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

/**
 * @author Peter
 * 
 */
public class GetServiceDurationBehaviour extends OneShotBehaviour {
	private static final long serialVersionUID = 1L;

	private BlackboardClient client;
	private ServiceStepMessage[] serviceSteps;

	/**
	 * @param a
	 */
	public GetServiceDurationBehaviour(Agent a, BlackboardClient client,
			ServiceStepMessage[] serviceSteps) {
		super(a);
		this.client = client;
		this.serviceSteps = serviceSteps;
	}

	public void action() {
		ObjectId serviceStepId;
		ACLMessage message;
		Agent agent = getAgent();
		Gson gson = new GsonBuilder().create();

		agent.addBehaviour(new GetServiceStepDuration(agent, client));
		try {
			for (ServiceStepMessage serviceStep : serviceSteps) {
				serviceStepId = client.insertDocument(BlackboardClient
						.<DBObject> parseJSONWithCheckException(gson
								.toJson(serviceStep)));

				message = new ACLMessage(ACLMessage.QUERY_IF);
//				message.addReceiver(r);
				//TODO add receiver to msg
				message.setContentObject(serviceStepId);
				message.setOntology("GetServiceStepDuration");
				agent.send(message);
			}
		} catch (InvalidDBNamespaceException | GeneralMongoException
				| InvalidJSONException | IOException e) {
			e.printStackTrace();
		}
	}
}
