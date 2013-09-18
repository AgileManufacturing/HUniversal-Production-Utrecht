/**
 * @file rexos/mas/equiplet_agent/behaviours/CanPerformStep.java
 * @brief Behaviour for handling the messages with the ontology CanPerformStep
 * @date Created: 2013-04-02
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
package rexos.mas.equiplet_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.Part;
import rexos.mas.data.ProductStep;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.ScheduleData;
import rexos.mas.data.StepStatusCode;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.utillities.log.LogLevel;
import rexos.utillities.log.Logger;

import com.mongodb.BasicDBObject;

/**
 * Receive behaviour for receiving messages with the ontology: "CanPerformStep".
 * Receives a message with a production step as content,
 * creates a productStep from that and places it on the blackboard.
 * Sends a message to his service agent with ontology: "CanDoProductionStep".
 * Starts a CanDoProductionStepResponse to wait on the response of the service agent.
 * When there is an error the agent sends a message to the sender with ACLMessage.FAILURE
 */
public class CanPerformStep extends ReceiveBehaviour {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate
	 *      The messageTemplate this behaviour listens to. This behaviour
	 *      listens to the ontology: CanPeformStep.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("CanPerformStep");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;

	/**
	 * @var BlackboardClient equipletBBClient
	 *      The blackboard client for the Equiplet.
	 **/
	private BlackboardClient equipletBBClient;

	/**
	 * Instantiates a new can perform step.
	 * 
	 * @param a The agent for this behaviour
	 * @param equipletBBClient The BlackboardClient for the EquipletBlackboard.
	 */
	public CanPerformStep(EquipletAgent a, BlackboardClient equipletBBClient) {
		super(a, messageTemplate);
		equipletAgent = a;
		this.equipletBBClient = equipletBBClient;
	}

	/**
	 * Function to handle the incoming messages for this behaviour. Handles the
	 * response to the CanPeformStep question and asks the service agent the
	 * same question.
	 * 
	 * @param message The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		//Logger.log("%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(),
				//message.getOntology());

		ProductionStep productStep = null;
		try {
			// gets the productstep out of the message.
			productStep = (ProductionStep) message.getContentObject();
			ObjectId productStepEntryId = null;

			// puts the productstep on the blackboard.
			// TODO: get inputParts instead of dummy data (overleggen met productagent)
			Part[] inputParts = {
					new Part(1), new Part(2)
			};
			ProductStep entry =
					new ProductStep(message.getSender(), productStep.getCapability(), productStep.getParameters(),
							inputParts, null, StepStatusCode.EVALUATING, new BasicDBObject(), new ScheduleData());
			productStepEntryId = equipletBBClient.insertDocument(entry.toBasicDBObject());
			equipletAgent.addCommunicationRelation(message.getConversationId(), productStepEntryId);

			// asks the service agent if the productionstep can be done.
			ACLMessage responseMessage = new ACLMessage(ACLMessage.REQUEST);
			responseMessage.setConversationId(message.getConversationId());
			responseMessage.addReceiver(equipletAgent.getServiceAgent());
			responseMessage.setOntology("CanDoProductionStep");
			responseMessage.setContentObject(productStepEntryId);
			equipletAgent.send(responseMessage);
		} catch(IOException | InvalidDBNamespaceException | GeneralMongoException | NullPointerException e) {
			Logger.log(LogLevel.ERROR, e);
			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.FAILURE);
			reply.setContent("Failed to process the step");
			equipletAgent.send(reply);
		} catch(UnreadableException e) {
			Logger.log(LogLevel.ERROR, e);
			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.FAILURE);
			reply.setContent("No step given");
			equipletAgent.send(reply);
		}
	}
}
