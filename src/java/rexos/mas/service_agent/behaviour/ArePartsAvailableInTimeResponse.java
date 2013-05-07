/**
 * @file ArePartsAvailableInTimeResponse.java
 * @brief 
 * @date Created: 23 apr. 2013
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
package rexos.mas.service_agent.behaviour;

import java.io.IOException;

import com.mongodb.BasicDBObject;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.equiplet_agent.StepStatusCode;
import rexos.mas.service_agent.ServiceAgent;

/**
 * @author Peter
 * 
 */
public class ArePartsAvailableInTimeResponse extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private String conversationId;
	private ServiceAgent agent;
	private ProductStepMessage productStep;

	/**
	 * @param a
	 */
	public ArePartsAvailableInTimeResponse(Agent a, String conversationId, ProductStepMessage productStep) {
		this(a, 2000, conversationId, productStep);
	}

	/**
	 * @param a
	 * @param millis
	 */
	public ArePartsAvailableInTimeResponse(Agent a, int millis,
			String conversationId, ProductStepMessage productStep) {
		super(a, millis, MessageTemplate.and(MessageTemplate
				.MatchConversationId(conversationId), MessageTemplate
				.MatchOntology("ArePartsAvailableInTimeResponse")));
		agent = (ServiceAgent) a;
		this.conversationId = conversationId;
		this.productStep = productStep;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				System.out.format("%s ArePartsAvailableInTimeResponse%n",
						agent.getLocalName());
				if (message.getPerformative() == ACLMessage.CONFIRM) {
					ACLMessage sendMsg = message.createReply();
					sendMsg.setOntology("GetPartsInfo");
					sendMsg.setPerformative(ACLMessage.QUERY_IF);
					sendMsg.setContentObject(productStep);
					agent.send(sendMsg);

					agent.addBehaviour(new GetPartsInfoResponse(agent,
							conversationId, productStep));
				} else {
					agent.getProductStepBBClient()
							.updateDocuments(
									new BasicDBObject("_id", productStep.get_id()),
									new BasicDBObject(
											"$set",
											new BasicDBObject("status",
													StepStatusCode.ABORTED.name())
													.append("statusData",
															new BasicDBObject(
																	"reason",
																	"productStep cannot be delivered on time"))));
				}
			} catch (IOException | InvalidDBNamespaceException
					| GeneralMongoException e) {
				e.printStackTrace();
				agent.doDelete();
			}
		} else {
			// TODO handle timeout
		}
	}
}
