/**
 * @file GetServiceStepBehaviour.java
 * @brief
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
 * 
 **/
package rexos.mas.service_agent.behaviour;

import jade.core.Agent;
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
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceStepMessage;

import com.mongodb.BasicDBObject;

/**
 * @author Peter
 * 
 */
public class GetServiceStepsDuration extends ReceiveOnceBehaviour {
	private static final long serialVersionUID = 1L;

	private String conversationId;
	private ServiceAgent agent;

	/**
	 * @param a
	 * @param serviceSteps
	 * @param conversationId
	 */
	public GetServiceStepsDuration(Agent a, String conversationId) {
		this(a, 2000, conversationId);
	}

	/**
	 * @param a
	 * @param millis
	 * @param serviceSteps
	 * @param conversationId
	 */
	public GetServiceStepsDuration(Agent a, int millis, String conversationId) {
		super(a, millis, MessageTemplate.and(MessageTemplate.MatchConversationId(conversationId),
				MessageTemplate.MatchOntology("GetServiceStepDurationResponse")));
		agent = (ServiceAgent) a;
		this.conversationId = conversationId;
	}

	/* (non-Javadoc)
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage) */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			try {
				BlackboardClient client = agent.getServiceStepBBClient();
				ServiceStepMessage serviceStep = new ServiceStepMessage();
				ObjectId nextStep = (ObjectId) message.getContentObject();
				int duration = 0;
				while(nextStep != null) {
					serviceStep.fromBasicDBObject((BasicDBObject) client.findDocumentById(nextStep));
					duration += serviceStep.getScheduleData().getDuration();
					nextStep = serviceStep.getNextStep();
				}

				ObjectId productStepId = serviceStep.getProductStepId();
				ProductStepMessage productStep =
						new ProductStepMessage((BasicDBObject) agent.getProductStepBBClient().findDocumentById(
								productStepId));
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
				e.printStackTrace();
			}
		} else {
			// TODO handle timeout
			Logger.log(agent.getName() + " - GetServiceStepDurationResponse timeout!");
			agent.doDelete();
		}
	}
}
