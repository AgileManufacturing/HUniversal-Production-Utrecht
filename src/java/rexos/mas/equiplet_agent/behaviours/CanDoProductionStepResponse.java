/**
 * @file rexos/mas/equiplet_agent/behaviours/CanDoProductionStepResponse.java
 * @brief Behaviour for handling the messages with the ontology CanDoProductionStepResponse
 * @date Created: 2013-04-02
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
package rexos.mas.equiplet_agent.behaviours;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.mas.equiplet_agent.ProductStep;

import com.mongodb.BasicDBObject;

/**
 * The Class CanDoProductionStepResponse.
 */
public class CanDoProductionStepResponse extends ReceiveOnceBehaviour {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate
	 *      The messageTemplate this behaviour listens to.
	 *      This behaviour listens to the ontology: CanDoProductionStepResponse.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate
			.MatchOntology("CanDoProductionStepResponse");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;
	private BlackboardClient equipletBBClient;

	/**
	 * Instantiates a new can do production step response.
	 * 
	 * @param a
	 *            - The agent for this behaviour
	 */
	public CanDoProductionStepResponse(Agent a,
			BlackboardClient equipletBBClient) {
		super(a, 5000, messageTemplate);
		equipletAgent = (EquipletAgent) a;
		this.equipletBBClient = equipletBBClient;
	}

	/**
	 * Function to handle the incoming messages for this behaviour.
	 * Handles the response to the CanPeformStep question and gives the result
	 * to the product agent.
	 * 
	 * @param message
	 *            - The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			Logger.log("%s received message from %s (%s)%n",
					myAgent.getLocalName(), message.getSender().getLocalName(),
					message.getOntology());

			ObjectId productStepEntryId = equipletAgent
					.getRelatedObjectId(message.getConversationId());
			try {
				// gets the productstep from the blackboard to get the
				// productAgent out of it.
				ProductStep productStep = new ProductStep(
						(BasicDBObject) equipletBBClient
								.findDocumentById(productStepEntryId));
				AID productAgent = productStep.getProductAgentId();
				// sends a message to the productAgent to answer to the question
				// CanPerformStep.
				ACLMessage responseMessage = new ACLMessage(
						message.getPerformative());
				responseMessage.setConversationId(message.getConversationId());
				responseMessage.setOntology("CanPerformStep");
				responseMessage.addReceiver(productAgent);
				if (message.getPerformative() == ACLMessage.DISCONFIRM) {
					// if the productstep can not be done by this equiplet
					// remove it.
					equipletBBClient.removeDocuments(new BasicDBObject("_id",
							productStepEntryId));
				}
				myAgent.send(responseMessage);
			} catch (Exception e) {
				// TODO: ERROR HANDLING
				Logger.log(e);
				myAgent.doDelete();
			}
		}
		myAgent.removeBehaviour(this);
	}
}
