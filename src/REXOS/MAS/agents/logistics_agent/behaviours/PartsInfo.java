/**
 * @file rexos/mas/logistics_agent/behaviours/PartsInfo.java
 * @brief Responds to PartsInfo messages, returning a mapping of part id to type and position.
 * @date Created: 22 apr. 2013
 * 
 * @author Peter Bonnema
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/
package agents.logistics_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

import com.mysql.jdbc.log.Log;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Queries;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.Part;
import agents.data_classes.Position;
import agents.logistics_agent.LogisticsAgent;
import agents.shared_behaviours.ReceiveOnceBehaviour;

/**
 * Responds to GetPartsInfo messages, returning a mapping of part id to type and position.
 */
public class PartsInfo extends ReceiveOnceBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate to match the messages.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("PartsInfo");
	
	/**
	 * @var LogisticsAgent logisticsAgent
	 *      The logisticsAgent of this behaviour.
	 */
	private LogisticsAgent logisticsAgent;
	
	/**
	 * Constructs the behaviour for the given agent.
	 * 
	 * @param a The agent associated with this behaviour.
	 * @param conversationId The conversationId that should be used for this behaviour.
	 */
	public PartsInfo(LogisticsAgent logisticsAgent, String conversationId) {
		this(logisticsAgent, 2000, conversationId);
	}

	/**
	 * Constructs the behaviour for the given agent.
	 * 
	 * @param a The agent associated with this behaviour.
	 * @param millis Timeout in milliseconds.
	 * @param conversationId The conversationId that should be used for this behaviour.
	 */
	public PartsInfo(LogisticsAgent logisticsAgent, int millis, String conversationId) {
		super(logisticsAgent, millis, MessageTemplate.and(MESSAGE_TEMPLATE,
				MessageTemplate.MatchConversationId(conversationId)));
		this.logisticsAgent = logisticsAgent;
		
	}
	

	/**
	 * Handles GetPartsInfo messages and responds with a GetPartsInfoResponse.
	 * 
	 * @see rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			try {
				Logger.log(LogLevel.DEBUG, "%s GetPartsInfo%n", logisticsAgent.getLocalName());
				Part[] parts = (Part[]) message.getContentObject();
				HashMap<Part, Position> partParameters = new HashMap<Part, Position>();
				
				int x = 2;
				int id = 1;
				int type = 3;
				
				for(Part part : parts) {
					Logger.log(LogLevel.DEBUG, "PartNo: " + part.getType());
					switch(part.getType()) {
					case 1: // Red ball
						// Grab a ball
					//	Iterator<Entry<Part, Position>> it = supplyCrateContent.entrySet().iterator();
					//	if(it.hasNext()) {
					//		Part ball = it.next().getKey();
					//		Position ballPosition = partParameters.remove(ball);
							
					//		partParameters.put(ball, ballPosition);
					//	}
						break;
					case 2: // Crate
						partParameters.put(logisticsAgent.getSupplyCrate(), new Position());
						partParameters.put(logisticsAgent.getProductCrate(), new Position());
						break;
					default:
						partParameters.put(new Part(part.getType(), id++),
								new Position((double)x++, 1.0, 3.0, new Part(type++, id + x)));
						break;					
					}
				}

				KnowledgeDBClient client = KnowledgeDBClient.getClient();
				int outputPartType = (int) client.executeSelectQuery(Queries.GET_PART_TYPE, "OutputPart")[0].get("id");
				int outputPartId = client.executeUpdateQuery(Queries.INSERT_PART, outputPartType);

				partParameters.put(new Part(outputPartType, outputPartId), null);

				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.INFORM);
				reply.setOntology("PartsInfo");
				reply.setContentObject(partParameters);
				logisticsAgent.send(reply);
			} catch (UnreadableException | IOException | KnowledgeException | KeyNotFoundException e) {
				Logger.log(LogLevel.ERROR, e);
				logisticsAgent.doDelete();
			}
		} else {
			logisticsAgent.removeBehaviour(this);
		}
	}
}
