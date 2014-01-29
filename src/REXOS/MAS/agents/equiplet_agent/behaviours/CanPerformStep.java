/**
 * @file rexos/mas/equiplet_agent/behaviours/CanPerformStep.java
 * @brief Behaviour for handling the messages with the ontology CanPerformStep
 * @date Created: 2013-04-02
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
package agents.equiplet_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.Part;
import agents.data_classes.ProductStep;
import agents.data_classes.ProductionStep;
import agents.data_classes.StepStatusCode;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

/**
 * Receive behaviour for receiving messages with the ontology: "CanPerformStep".
 * Receives a message with a production step as content,
 * creates a productStep from that and places it on the blackboard.
 * Sends a message to his service agent with ontology: "CanDoProductionStep".
 * Starts a CanDoProductionStepResponse to wait on the response of the service agent.
 * When there is an error the agent sends a message to the sender with ACLMessage.FAILURE
 */
public class CanPerformStep extends ReceiveBehaviour implements ParentBehaviourCallback{
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate this behaviour listens to. This behaviour
	 *      listens to the ontology: CanPeformStep.
	 */
	
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("CanPerformStep");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;

	/**
	 * @var BlackboardClient productStepsBlackBoard
	 *      The productStepsBlackBoard client for the Equiplet.
	 **/
	private BlackboardClient productStepsBlackboard;

	/**
	 * @var ProductStep currentProductStep
	 * 		The used prodcut step in the current question
	 */
	private ProductStep currentProductStep;
	
	/**
	 * Instantiates a new can perform step.
	 * 
	 * @param equipletAgent The agent for this behaviour
	 * @param productStepsBlackBoard The BlackboardClient for the EquipletBlackboard.
	 */
	public CanPerformStep(EquipletAgent equipletAgent, BlackboardClient productStepsBlackBoard) {
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent = equipletAgent;
		this.productStepsBlackboard = productStepsBlackBoard;
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

		if(message.getPerformative() == ACLMessage.QUERY_IF){
			
			try {
				// gets the productstep out of the message.
				ProductionStep productStep = null;
				ObjectId productStepEntryId = null;
				
				productStep = (ProductionStep) message.getContentObject();
	
				// puts the productstep on the blackboard.
				// TODO: get inputParts instead of dummy data (overleggen met productagent)
				
				// Set up inputParts depending on capability
				Part[] inputParts = null;
				
				if(productStep != null) {
					if(productStep.getCapability() == 1) {
						inputParts = new Part[]{
							new Part(1), new Part(2)
						};
					} else if(productStep.getCapability() == 3) {
						inputParts = new Part[]{
							new Part(3)
						};
					}
				} else {
					// error
				}
				currentProductStep =
						new ProductStep(message.getSender(), productStep.getCapability(), productStep.getParameters(),
								inputParts, null, StepStatusCode.EVALUATING, new BasicDBObject());
				productStepEntryId = productStepsBlackboard.insertDocument(currentProductStep.toBasicDBObject());
				equipletAgent.addCommunicationRelation(message.getConversationId(), productStepEntryId);
				
				// start behaviour to ask the service agent if the productionstep can be done.
				equipletAgent.addBehaviour(new CanPerformProductionStep(equipletAgent, productStepsBlackboard, this,
																		message.getConversationId(), productStepEntryId));
				
				
				
				
			} catch(InvalidDBNamespaceException | GeneralMongoException | NullPointerException e) {
				Logger.log(LogLevel.ERROR, "", e);
				ACLMessage errorResponse = message.createReply();
				errorResponse.setPerformative(ACLMessage.FAILURE);
				errorResponse.setContent("Failed to process the step");
				equipletAgent.send(errorResponse);
			} catch(UnreadableException e) {
				Logger.log(LogLevel.ERROR, "", e);
				ACLMessage errorResponse = message.createReply();
				errorResponse.setPerformative(ACLMessage.FAILURE);
				errorResponse.setContent("No step given");
				equipletAgent.send(errorResponse);
			}
		}
		else if (message.getPerformative() == ACLMessage.DISCONFIRM){
			
		}
	}

	/**
	 * Function to handle the callback from subbehaviours
	 * @param message The message received in the subbehaviour
	 * @param arguments Optional arguments from the subbehaviour
	 */
	@Override
	public void callback(ACLMessage message, BehaviourCallbackItem arguments){
		// sends a message with the result of the behaviour subroutine
		// to the productAgent to answer to the question CanPerformStep.
		ACLMessage resultMessage = new ACLMessage(message.getPerformative());
		resultMessage.setConversationId(message.getConversationId());
		resultMessage.setOntology("CanPerformStep");
		resultMessage.addReceiver(currentProductStep.getProductAgentId());
		equipletAgent.send(resultMessage);
	}
}
