/**
 * @file GetServiceStepBehaviour.java
 * @brief 
 * @date Created: 11 apr. 2013
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
import java.util.Arrays;
import java.util.List;

import org.bson.types.ObjectId;

import rexos.mas.behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.service_agent.ServiceAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * @author Peter
 * 
 */
public class GetServiceDuration extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private BlackboardClient productionStepBlackBoard, serviceStepBlackBoard;
	private List<BasicDBObject> serviceSteps;
	private String conversationId;
	private ServiceAgent agent;

	private long duration = 0;
	private int remainingServiceSteps;

	/**
	 * @param a
	 */
	public GetServiceDuration(Agent a,
			BlackboardClient productionStepBlackBoard,
			BlackboardClient serviceStepBlackBoard,
			BasicDBObject[] serviceSteps, String conversationId) {
		this(a, 2000, productionStepBlackBoard, serviceStepBlackBoard,
				serviceSteps, conversationId);
	}

	/**
	 * @param a
	 */
	public GetServiceDuration(Agent a, int millis,
			BlackboardClient productionStepBlackBoard,
			BlackboardClient serviceStepBlackBoard,
			BasicDBObject[] serviceSteps, String conversationId) {
		super(
				a,
				millis,
				MessageTemplate
						.and(MessageTemplate
								.MatchConversationId(conversationId),
								MessageTemplate.or(
										MessageTemplate
												.MatchOntology("GetServiceStepDurationResponse"),
										MessageTemplate
												.MatchOntology("GetTransportDurationResponse"))));

		this.productionStepBlackBoard = productionStepBlackBoard;
		this.serviceStepBlackBoard = serviceStepBlackBoard;
		this.serviceSteps = Arrays.asList(serviceSteps);
		this.conversationId = conversationId;
		agent = (ServiceAgent) a;

		remainingServiceSteps = serviceSteps.length;
	}

	@Override
	public void onStart() {
		System.out.format("%s asking %s for duration of %d steps%n", agent
				.getLocalName(), agent.getHardwareAgentAID().getLocalName(),
				serviceSteps.size());

		ObjectId serviceStepId;
		ACLMessage message = new ACLMessage(ACLMessage.QUERY_IF);
		message.addReceiver(agent.getHardwareAgentAID());
		message.setOntology("GetServiceStepDuration");
		message.setConversationId(conversationId);
		try {
			for (BasicDBObject serviceStep : serviceSteps) {
				serviceStepId = serviceStepBlackBoard.insertDocument(serviceStep);
				message.setContentObject(serviceStepId);
				agent.send(message);
			}
		} catch (InvalidDBNamespaceException | GeneralMongoException
				| IOException e) {
			e.printStackTrace();
			agent.doDelete();
		}

		message.clearAllReceiver();
		message.addReceiver(agent.getLogisticsAID());
		message.setOntology("GetTransportDuration");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				DBObject serviceStep = serviceStepBlackBoard
						.findDocumentById((ObjectId) message.getContentObject());
				ProductStepMessage productStep = new ProductStepMessage(
						(BasicDBObject)productionStepBlackBoard.findDocumentById(
								(ObjectId) serviceStep.get("productStepId")));
				BasicDBObject scheduleData;
				
				switch (message.getOntology()) {
				case "GetServiceStepDurationResponse":
					System.out.format("%s says step type %s will take %d%n",
							agent.getLocalName(), serviceStep.get("type"),
							((BasicDBObject) serviceStep.get("scheduleData"))
									.getLong("duration"));

					scheduleData = (BasicDBObject) serviceStep
							.get("scheduleData");
					duration += scheduleData.getLong("duration");
					if (--remainingServiceSteps == 0) {
						ACLMessage answer = new ACLMessage(ACLMessage.QUERY_IF);
						answer.addReceiver(agent.getLogisticsAID());
						answer.setConversationId(conversationId);
						answer.setOntology("GetTransportDuration");

						Long[] inputParts = productStep.getInputParts();
						answer.setContentObject(inputParts);

						agent.send(answer);
					}
					break;
				case "GetTransportDurationResponse":
					duration += Integer.parseInt(message.getContent());

					// read scheduleData from product step BB
					scheduleData = productStep.getScheduleData().toBasicDBObject();
					scheduleData.put("duration", duration);
					productionStepBlackBoard.updateDocuments(new BasicDBObject(
							"_id", serviceStep.get("productStepId")),
							new BasicDBObject("$set", new BasicDBObject("scheduleData", scheduleData)));

					System.out.format(
							"Saving duration of %d in prod. step %s%n",
							duration, serviceStep.get("productStepId"));

					ACLMessage answer = new ACLMessage(ACLMessage.INFORM);
					answer.addReceiver(agent.getEquipletAgentAID());
					answer.setConversationId(conversationId);
					answer.setOntology("ProductionDurationResponse");
					agent.send(answer);

					System.out.format("%s sending msg (%s)%n",
							myAgent.getLocalName(), answer.getOntology());

					agent.removeBehaviour(this);
					break;
				default:
					break;
				}
			} catch (IOException | InvalidDBNamespaceException
					| GeneralMongoException | UnreadableException e) {
				e.printStackTrace();
			}
		} else {
			// TODO handle timeout
		}
	}
}
