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
package serviceAgent.behaviour;

import java.io.IOException;
import java.lang.reflect.Type;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.bson.types.ObjectId;

import behaviours.ReceiveBehaviour;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;
import com.google.gson.JsonSyntaxException;
import com.google.gson.TypeAdapter;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import equipletAgent.StepStatusCode;

import newDataClasses.ScheduleData;
import nl.hu.client.BlackboardClient;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;
import nl.hu.client.InvalidJSONException;
import serviceAgent.ServiceAgent;
import serviceAgent.ServiceStepMessage;
import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
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
	private List<ServiceStepMessage> serviceSteps;
	private String conversationId;

	private long duration = 0;

	/**
	 * @param a
	 */
	public GetServiceDuration(Agent a,
			BlackboardClient productionStepBlackBoard,
			BlackboardClient serviceStepBlackBoard,
			ServiceStepMessage[] serviceSteps, String conversationId) {
		this(a, 2000, productionStepBlackBoard, serviceStepBlackBoard,
				serviceSteps, conversationId);
	}

	/**
	 * @param a
	 */
	public GetServiceDuration(Agent a, int millis,
			BlackboardClient productionStepBlackBoard,
			BlackboardClient serviceStepBlackBoard,
			ServiceStepMessage[] serviceSteps, String conversationId) {
		super(a, millis, MessageTemplate
				.and(MessageTemplate
						.MatchOntology("GetServiceStepDurationResponse"),
						MessageTemplate.MatchConversationId(conversationId)));
		this.productionStepBlackBoard = productionStepBlackBoard;
		this.serviceStepBlackBoard = serviceStepBlackBoard;
		this.serviceSteps = Arrays.asList(serviceSteps);
		this.conversationId = conversationId;
	}

	public void onStart() {
		ObjectId serviceStepId;
		ACLMessage message;
		ServiceAgent agent = (ServiceAgent) getAgent();
		Gson gson = new GsonBuilder().registerTypeAdapter(DBObject.class,
				new InstanceCreator<DBObject>() {
					@Override
					public DBObject createInstance(Type type) {
						return new BasicDBObject();
					}
				}).create();

		try {
			System.out.format("%s asking %s for duration of %d steps%n", agent
					.getLocalName(),
					agent.getHardwareAgentAID().getLocalName(), serviceSteps
							.size());
			for (ServiceStepMessage serviceStep : serviceSteps) {
				serviceStepId = serviceStepBlackBoard.insertDocument(gson.toJson(serviceStep));

				message = new ACLMessage(ACLMessage.QUERY_IF);
				message.addReceiver(agent.getHardwareAgentAID());
				message.setOntology("GetServiceStepDuration");
				message.setContentObject(serviceStepId);
				message.setConversationId(conversationId);
				agent.send(message);
			}
		} catch (InvalidDBNamespaceException | GeneralMongoException
				| IOException | InvalidJSONException e) {
			e.printStackTrace();
			agent.doDelete();
		}
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
				Gson gson = new GsonBuilder().create();
				ServiceStepMessage serviceStep = gson.fromJson(gson
						.toJson(serviceStepBlackBoard
								.findDocumentById((ObjectId) message
										.getContentObject())),
						ServiceStepMessage.class);

				System.out.format("%s says step type %s will take %d %n",
						getAgent().getLocalName(), serviceStep.getType(),
						serviceStep.getScheduleData().getDuration());

				duration += serviceStep.getScheduleData().getDuration();
				serviceSteps.remove(serviceStep);
				// TODO store duration when ready
				if (serviceSteps.isEmpty()) {
					ScheduleData scheduleData = null;

					productionStepBlackBoard.updateDocuments(
							new BasicDBObject("_id", serviceStep
									.getProductStepId()),
							new BasicDBObject("$set", new BasicDBObject(
									"scheduleData", gson.fromJson(
											gson.toJson(scheduleData),
											ScheduleData.class))));
					getAgent().removeBehaviour(this);
				}
			} catch (UnreadableException | JsonSyntaxException
					| InvalidDBNamespaceException | GeneralMongoException e) {
				e.printStackTrace();
			}
		} else {
			// TODO handle timeout
		}
	}
}
