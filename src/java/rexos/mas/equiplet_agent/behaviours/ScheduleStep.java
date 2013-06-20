/**
 * @file rexos/mas/equiplet_agent/behaviours/ScheduleStep.java
 * @brief Behaviour for handling the messages with the ontology ScheduleStep
 * @date Created: 2013-04-02
 * 
 * @author Wouter Veen
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
package rexos.mas.equiplet_agent.behaviours;

import java.io.IOException;
import java.util.List;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.EquipletAgent;
import rexos.mas.equiplet_agent.ProductStep;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

/**
 * Receive behaviour for receiving messages with the ontology: "ScheduleStep".
 * When it receives this message, it checks if it is possible to do plan the step on the given timeslot.
 * Sends an <code>ACLMessage.DISCONFIRM</code> when it is not possible. When its possible it sends a message
 * to the service agent with ontology: "ScheduleStep". and updates the blackboard.
 */
public class ScheduleStep extends ReceiveBehaviour {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = -3574738583814321426L;

	/**
	 * @var MessageTemplate messageTemplate
	 *      The messageTemplate this behaviour listens to. This behaviour
	 *      listens to the ontology: ScheduleStep.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("ScheduleStep");

	/**
	 * @var EquipletAgent equipletAgent
	 *      The equipletAgent related to this behaviour.
	 */
	private EquipletAgent equipletAgent;

	/**
	 * @var Blackboard productBBClient
	 *      The productBBClient for this behaviour.
	 */
	private BlackboardClient productBBClient;

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a
	 *      The agent for this behaviour
	 * @param productBBClient
	 * 		The blackboardClient for the productSteps.
	 */
	public ScheduleStep(Agent a, BlackboardClient productBBClient) {
		super(a, messageTemplate);
		equipletAgent = (EquipletAgent) a;
		this.productBBClient = productBBClient;
	}

	/**
	 * Function to handle the incoming messages for this behaviour. Handles the response to the ScheduleStep question
	 * and asks the service agent to schedule.
	 * 
	 * @param message
	 *            - The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		Logger.log("%s received message from %s%n", equipletAgent.getLocalName(), message.getSender().getLocalName(),
				message.getOntology());

		try {
			// Gets the timeslot out of the message content.
			int start = (Integer) message.getContentObject();
			Logger.log("Equiplet agent - scheduling step for timeslot %d%n", start);

			// Gets the scheduledata out of the productstep.
			ObjectId productStepId = equipletAgent.getRelatedObjectId(message.getConversationId());
			ScheduleData scheduleData =
					new ScheduleData(
							(BasicDBObject) (productBBClient.findDocumentById(productStepId)).get("scheduleData"));
			int end = start + scheduleData.getDuration();

			// Gets planned steps
			List<DBObject> plannedSteps =
					productBBClient.findDocuments(QueryBuilder.start("scheduleData.startTime").greaterThan(-1).get());

			boolean fitsInSchedule = true;

			// check if other steps not are scheduled.
			for(DBObject plannedStep : plannedSteps) {
				ProductStep productStep = new ProductStep((BasicDBObject) plannedStep);
				ScheduleData stepScheduleData = productStep.getScheduleData();

				int scheduledStepStart = stepScheduleData.getStartTime();
				int scheduledStepEnd = scheduledStepStart + stepScheduleData.getDuration();

				if(start >= scheduledStepStart && start <= scheduledStepEnd) {
					fitsInSchedule = false;
				} else if(end >= scheduledStepStart && end <= scheduledStepEnd) {
					fitsInSchedule = false;
				} else if(start <= scheduledStepStart && end >= scheduledStepEnd) {
					fitsInSchedule = false;
				}
			}
			if(fitsInSchedule) {
				scheduleData.setStartTime(start);
				productBBClient.updateDocuments(new BasicDBObject("_id", productStepId), new BasicDBObject("$set",
						new BasicDBObject("scheduleData", scheduleData.toBasicDBObject())));

				ACLMessage scheduleMessage = new ACLMessage(ACLMessage.REQUEST);
				scheduleMessage.addReceiver(equipletAgent.getServiceAgent());
				scheduleMessage.setOntology("ScheduleStep");
				scheduleMessage.setContentObject(productStepId);
				scheduleMessage.setConversationId(message.getConversationId());
				equipletAgent.send(scheduleMessage);
			} else {
				Logger.log("ScheduleStep disconfirm");
				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.DISCONFIRM);
				myAgent.send(reply);
			}
		} catch(IOException | InvalidDBNamespaceException | GeneralMongoException | UnreadableException e) {
			Logger.log(e);
		}
	}
}
