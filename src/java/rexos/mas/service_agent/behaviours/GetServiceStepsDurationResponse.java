/**
 * @file rexos/mas/service_agent/behaviours/GetServiceStepsDurationResponse.java
 * @brief Handles the GetServiceStepsDurationResponse message from the hardwareAgent that indicates that the duration of
 *        all serviceSteps have been saved on the serviceStepsBlackBoard.
 * @date Created: 11 apr. 2013
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
package rexos.mas.service_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.data.ScheduleData;
import rexos.mas.equiplet_agent.ProductStep;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceStep;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the GetServiceStepsDurationResponse message. The hardwareAgent sends this message in response
 * to the GetServiceStepsDurationResponse message to indicate that it saved the duration of all serviceSteps on the
 * serviceStepBlackboard. It will generate a timeout after a specified period if no message is received.
 * 
 * @author Peter
 * 
 */
public class GetServiceStepsDurationResponse extends ReceiveOnceBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -3454535337099894852L;

	/**
	 * @var String conversationId
	 *      The conversationId which the answer will have. Any messages send in response will also have this
	 *      conversationId.
	 */
	private String conversationId;

	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent agent;

	/**
	 * Creates a new GetServiceStepsDurationResponse instance with the specified parameters. A default value of 2000 ms
	 * is used for the timeout.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 */
	public GetServiceStepsDurationResponse(ServiceAgent agent, String conversationId) {
		this(agent, 2000, conversationId);
	}

	/**
	 * Creates a new GetServiceStepsDurationResponse instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 * @param millis the timeout period in milliseconds.
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 */
	public GetServiceStepsDurationResponse(ServiceAgent agent, int millis, String conversationId) {
		super(agent, millis, MessageTemplate.and(MessageTemplate.MatchConversationId(conversationId),
				MessageTemplate.MatchOntology("GetServiceStepDurationResponse")));
		this.agent = agent;
		this.conversationId = conversationId;
	}

	/**
	 * Handles an incoming message from the hardwareAgent. The message indicates that the duration of the serviceStep
	 * specified by the GetServiceStepsDurationResponse message and all serviceSteps linked to it are filled in on the
	 * serviceStepsBlackBoard. First the duration of all serviceSteps are summed up and the resulting duration is saved
	 * as the duration of the productStep on the productStepBlackBoard. Then a ProductionDurationResponse message is
	 * send to the equipletAgent to inform it that the duration of the productStep has been determined. If a timeout
	 * occurs this method will call doDelete() on the service agent.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			try {
				BlackboardClient client = agent.getServiceStepBBClient();
				ServiceStep serviceStep = new ServiceStep();
				ObjectId nextStep = (ObjectId) message.getContentObject();
				int duration = 0;
				while(nextStep != null) {
					serviceStep.fromBasicDBObject((BasicDBObject) client.findDocumentById(nextStep));
					duration += serviceStep.getScheduleData().getDuration();
					nextStep = serviceStep.getNextStep();
				}

				ObjectId productStepId = serviceStep.getProductStepId();
				ProductStep productStep =
						new ProductStep((BasicDBObject) agent.getProductStepBBClient().findDocumentById(productStepId));
				ScheduleData scheduleData = productStep.getScheduleData();
				scheduleData.setDuration(duration);

				Logger.log("Saving duration of %d in prod. step %s%n", duration, productStepId);
				agent.getProductStepBBClient().updateDocuments(new BasicDBObject("_id", productStepId),
						new BasicDBObject("$set", new BasicDBObject("scheduleData", scheduleData.toBasicDBObject())));

				ACLMessage answer = new ACLMessage(ACLMessage.INFORM);
				answer.addReceiver(agent.getEquipletAgentAID());
				answer.setConversationId(conversationId);
				answer.setOntology("ProductionDurationResponse");
				agent.send(answer);

				Logger.log("%s sending msg (%s)%n", myAgent.getLocalName(), answer.getOntology());
			} catch(InvalidDBNamespaceException | GeneralMongoException | UnreadableException e) {
				Logger.log(e);
			}
		} else {
			Logger.log(agent.getName() + " - GetServiceStepDurationResponse timeout!");
			agent.doDelete();
		}
	}
}
