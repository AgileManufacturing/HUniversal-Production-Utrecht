/**
 * @file src/REXOS/MAS/agents/equiplet_agent/behaviours/ProductionDuration.java
 * @brief Behaviour for handling the messages with the ontology ProductionDuration
 * @date Created: 2013-14-10
 * 
 * @author Hessel Meulenbeld
 * 		   Roy Scheefhals
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
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
 **/
package agents.equiplet_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;

import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.data_classes.ScheduleData;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;

/**
 * The receive behaviour for receiving messages with ontology: "ProductionDuration".
 * When a message is received it checks if it can handle it.
 * If not it sends a message to the sender with ontology:"ConversationIdUnknown".
 * When it can handle the message, it sends a message to the serviceAgent with ontology: "ProductionDuration".
 */
public class ProductionDuration extends ReceiveBehaviour implements ParentBehaviourCallback {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 2502684423295372637L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate this behaviour listens to.
	 *      This behaviour listens to the ontology: ProductionDuration.
	 */
	private static MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("ProductionDuration");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;

	/**
	 * Instantiates a new gets the production duration.
	 * 
	 * @param a The agent for this behaviour
	 * 
	 */
	public ProductionDuration(EquipletAgent equipletAgent) {
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent = equipletAgent;
	}

	/**
	 * Function to handle the incoming messages for this behaviour.
	 * Handles the response to the ProductionDuration question and asks the service agent the same question.
	 * 
	 * @param message - The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		Logger.log(LogLevel.DEBUG, "%s received message from %s%n", equipletAgent.getLocalName(), message.getSender().getLocalName(),
				message.getOntology());
		try {
			// gets the productstepId and sends it to the service agent with the ontology GetProductionStepDuration.
			ObjectId productStepId = equipletAgent.getRelatedObjectId(message.getConversationId());
			if(productStepId == null) {
				Logger.log(LogLevel.DEBUG, "Conversation id not known");
				ACLMessage responseMessage = message.createReply();
				responseMessage.setPerformative(ACLMessage.DISCONFIRM);
				//TODO: why change the ontology ?
				responseMessage.setOntology("ConversationIdUnknown");
				equipletAgent.send(responseMessage);
			} else {
				
				equipletAgent.addBehaviour(new ProductStepDuration(equipletAgent, equipletAgent.getProductStepBBClient(),
																	this, message.getConversationId(), productStepId));
		//		ProductionDurationResponse productionDurationResponseBehaviour =
		//				new ProductionDurationResponse(myAgent, equipletAgent.getProductStepBBClient());
		//		myAgent.addBehaviour(productionDurationResponseBehaviour);

		//		ACLMessage responseMessage = new ACLMessage(ACLMessage.QUERY_REF);
		//		responseMessage.addReceiver(equipletAgent.getServiceAgent());
		//		responseMessage.setConversationId(message.getConversationId());
		//		responseMessage.setContentObject(productStepId);
		//		responseMessage.setOntology("ProductStepDuration");
		//		myAgent.send(responseMessage);
			}
		} catch(Exception e) {
			Logger.log(LogLevel.ERROR, "", e);
			equipletAgent.doDelete();
		}
	}

	@Override
	public void callback(ACLMessage result, BehaviourCallbackItem arguments) {
		// sends a message to the productAgent with the production
		// duration.
		
		ProductStep productStep = (ProductStep)arguments.getArgument("productStep");
		ScheduleData schedule = (ScheduleData)arguments.getArgument("schedule");
		
		ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
		responseMessage.addReceiver(productStep.getProductAgentId());
		responseMessage.setOntology("ProductionDuration");
		responseMessage.setConversationId(result.getConversationId());
		try {
			responseMessage.setContentObject(new Long(schedule.getDuration()));
		} catch (IOException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}
		equipletAgent.send(responseMessage);
		
	}
}
