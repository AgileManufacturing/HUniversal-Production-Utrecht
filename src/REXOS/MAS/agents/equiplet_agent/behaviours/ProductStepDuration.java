/**
 * @file rexos/mas/equiplet_agent/behaviours/GetProductionDuration.java
 * @brief redirecting Behaviour for handling the messages with the ontology ProductStepDuration
 * @date Created: 2013-04-02
 * 
 * @author  Roy Scheefhals
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

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;
import java.io.Serializable;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.data_classes.ScheduleData;
import agents.equiplet_agent.EquipletAgent;
import agents.shared_behaviours.ReceiveBehaviour;
import agents.shared_behaviours.ReceiveOnceBehaviour;

import com.mongodb.BasicDBObject;

/**
 * A receive once behaviour for receiving messages with the ontology: "ProductionDurationResponse".
 * Receives the message from its service agent and sends the productduration to the product agent.
 * The message send to the product agent has the ontology: "ProductionDuration".
 */
public class ProductStepDuration extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 **/
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The MessageTemplate indicating the messages this behaviour wishes to
	 *      receive.
	 **/
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("ProductStepDuration");

	/**
	 * @var EquipletAgent equipletAgent
	 *      A reference to the equipletAgent this behaviour belongs to.
	 **/
	private EquipletAgent equipletAgent;

	/**
	 * @var BlackboardClient productStepsBlackboard
	 *      The blackboardclient for this equiplet's producutStepBlackboard.
	 **/
	private BlackboardClient productStepsBlackboard;

	/**
	 * @var ParentBehaviourCallback parentBehaviourCallback
	 * 		The parentbehaviour callback this redirect calls back to
	 */
	private ParentBehaviourCallback parentBehaviourCallback;

	/**
	 * @var ObjectId objectId
	 * 		The ObjectId used for the message
	 */
	private ObjectId objectId;

	/**
	 * @var String conversationId
	 * 		The conversationId used for the message
	 */
	private String conversationId;

	/**
	 * Instantiates a new production duration response.
	 * 
	 * @param equipletAgent
	 *            the equipletagent of this behaviour
	 * @param productStepsBlackboard
	 *            the productStepsblackboard of the equiplet.
	 */
	public ProductStepDuration(EquipletAgent equipletAgent, BlackboardClient productStepsBlackboard, ParentBehaviourCallback parentBehaviourCallback,
			String conversationID, ObjectId objectId) {
		super(equipletAgent, MESSAGE_TEMPLATE);
		this.equipletAgent =  equipletAgent;
		this.productStepsBlackboard = productStepsBlackboard;
		
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.objectId = objectId;
		this.conversationId = conversationID;
	}
	
	@Override
	public void onStart(){
		ACLMessage responseMessage = new ACLMessage(ACLMessage.QUERY_REF);
		responseMessage.addReceiver(equipletAgent.getServiceAgent());
		responseMessage.setConversationId(conversationId);
		if (objectId != null){
			try {
				responseMessage.setContentObject(objectId);
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		}
		responseMessage.setOntology("ProductStepDuration");
		equipletAgent.send(responseMessage);
	}

	/**
	 * Function for handling the incoming of messages.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			//Logger.log("%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(),
					//message.getOntology());

			try {
				// gets the productstep
				ObjectId productStepId = equipletAgent.getRelatedObjectId(message.getConversationId());
				ProductStep productStep = new ProductStep((BasicDBObject) productStepsBlackboard.findDocumentById(productStepId));

				long duration = (Long) message.getContentObject();
				
				ScheduleData schedule = productStep.getScheduleData();
//				schedule.setDuration(schedule.getDuration() + (6000/equipletAgent.getTimer().getTimeSlotLength()));
//				equipletBBClient.updateDocuments(new BasicDBObject("_id", id),
//												new BasicDBObject("$set", new BasicDBObject("scheduleData.duration", schedule.getDuration())));
				BehaviourCallbackItem scheduleArguments = new BehaviourCallbackItem();
				scheduleArguments.addArgument("schedule", schedule);
				scheduleArguments.addArgument("productStep", productStep);
				scheduleArguments.addArgument("duration", duration);
				parentBehaviourCallback.callback(message, scheduleArguments);
				equipletAgent.removeBehaviour(this);
				
		//		Logger.log(LogLevel.DEBUG, "sending message: %s%n", responseMessage.getOntology());
			} catch(InvalidDBNamespaceException | GeneralMongoException | UnreadableException e) {
				Logger.log(LogLevel.ERROR, e);
				equipletAgent.doDelete();
			}
		}else{
			Logger.log(LogLevel.ERROR, "equiplet agent - duration response timed out");
		}
	}
}
