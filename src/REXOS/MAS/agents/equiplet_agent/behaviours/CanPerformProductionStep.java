/**
* @file src/REXOS/MAS/agents/equiplet_agent/behaviours/CanPerformProductionStep.java
* @brief Redirecting behaviour for handling the messages with the ontology CanPerformProductionStep
* @date Created: 2013-04-02
*
* @author Hessel Meulenbeld
* 		  Roy Scheefhals
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

package agents.equiplet_agent.behaviours;

import java.io.IOException;
import java.io.Serializable;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

public class CanPerformProductionStep extends ReceiveBehaviour {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate this behaviour listens to.
	 *      This behaviour listens to the ontology: CanPerformProductionStep.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("CanPerformProductionStep");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;
	/**
	 * @var BlackboardClient productStepsBlackboard
	 * 		BlackboardClient for the product step blackboard.
	 */
	private BlackboardClient productStepsBlackboard;

	/**
	 * @var ParentBehaviourCallback parentBehaviourCallback
	 * 		The behaviour this behaviour calls back to
	 */
	private ParentBehaviourCallback parentBehaviourCallback;
	
	/**
	 * @var ObjectId objectId
	 * 		The ObjectId used to check the production step
	 */
	private ObjectId objectId;
	
	/**
	 * @var String conversationID
	 * 		The ConversationId used for the message sent
	 */
	private String conversationId;
	
	/**
	 * Instantiates a new can do production step response.
	 * 
	 * @param equipletAgent
	 *      The agent for this behaviour
	 * @param productStepBlackBoard
	 * 		BlackboardClient for the product step blackboard.            
	 * @throws IOException Throws IOException when contentObject cannot be serialized
	 */
	public CanPerformProductionStep(EquipletAgent equipletAgent,
			BlackboardClient productStepsBlackboard, ParentBehaviourCallback parentBehaviourCallback,
			String conversationID, ObjectId objectId){
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent =  equipletAgent;
		this.productStepsBlackboard = productStepsBlackboard;
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.objectId = objectId;
		this.conversationId = conversationID;
	}
	
	@Override
	public void onStart(){
		ACLMessage queryIFMessage = new ACLMessage(ACLMessage.QUERY_IF);
		queryIFMessage.setConversationId(conversationId);
		queryIFMessage.addReceiver(equipletAgent.getServiceAgent());
		queryIFMessage.setOntology("CanPerformProductionStep");
		if (objectId != null){
			try {
				queryIFMessage.setContentObject(objectId);
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, "", e);
			}
		}
		equipletAgent.send(queryIFMessage);
	
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

			ObjectId productStepEntryId = equipletAgent
					.getRelatedObjectId(message.getConversationId());
			
			if (message.getPerformative() == ACLMessage.DISCONFIRM){
				try{
				productStepsBlackboard.removeDocuments(new BasicDBObject("_id",
						productStepEntryId));
				}catch (GeneralMongoException | InvalidDBNamespaceException e) {
					Logger.log(LogLevel.ERROR, "", e);
				}
			}
			
			//callback to the parent behaviour with the message
			parentBehaviourCallback.callback(message, null);
			equipletAgent.removeBehaviour(this);
				// gets the productstep from the blackboard to get the
				// productAgent out of it.
				
		
	}
}