/**
 * @file rexos/mas/logistics_agent/behaviours/GetPartsInfo.java
 * @brief Responds to GetPartsInfo messages, returning a mapping of part id to type and position.
 * @date Created: 22 apr. 2013
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
package rexos.mas.logistics_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;
import java.util.HashMap;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.data.Part;
import rexos.mas.data.Position;

/**
 * Responds to GetPartsInfo messages, returning a mapping of part id to type and position.
 */
public class GetPartsInfo extends ReceiveOnceBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * Constructs the behaviour for the given agent.
	 * @param a The agent associated with this behaviour.
	 * @param conversationId The conversationId that should be used for this behaviour.
	 */
	public GetPartsInfo(Agent a, String conversationId) {
		this(a, 2000, conversationId);
	}

	/**
	 * Constructs the behaviour for the given agent.
	 * @param a The agent associated with this behaviour.
	 * @param millis Timeout in milliseconds.
	 * @param conversationId The conversationId that should be used for this behaviour.
	 */
	public GetPartsInfo(Agent a, int millis, String conversationId) {
		super(a, millis, MessageTemplate.and(
				MessageTemplate.MatchOntology("GetPartsInfo"),
				MessageTemplate.MatchConversationId(conversationId)));
	}

	/**
	 * Handles GetPartsInfo messages and responds with a GetPartsInfoResponse.
	 * 
	 * @see rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				Logger.log("%s GetPartsInfo%n", myAgent.getLocalName());
				Part[] parts = (Part[]) message.getContentObject();				
				HashMap<Part, Position> partParameters = new HashMap<Part, Position>();
				int x = 2;
				int id = 1;
				int type = 3;
				for (Part part : parts) {
					partParameters.put(new Part(part.getType(), id++), new Position(x++, 1, 3, new Part(type++, id + x)));
				}

				KnowledgeDBClient client = KnowledgeDBClient.getClient();
				int outputPartType = client.executeUpdateQuery(Queries.INSERT_PART_TYPE, new Object[]{"OutputPart", parts.toString()});
				int outputPartId = client.executeUpdateQuery(Queries.INSERT_PART, new Object[]{outputPartType});
				
				partParameters.put(new Part(outputPartType, outputPartId), null);
				
				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.INFORM);
				reply.setOntology("GetPartsInfoResponse");
				reply.setContentObject(partParameters);
				myAgent.send(reply);
			} catch (UnreadableException | IOException | KnowledgeException e) {
				Logger.log(e);
				myAgent.doDelete();
			}
		} else {
			myAgent.removeBehaviour(this);
		}
	}
}
