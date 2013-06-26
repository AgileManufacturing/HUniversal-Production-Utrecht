/**
 * @file rexos/mas/service_agent/behaviours/GetPartsInfoResponse.java
 * @brief Handles the GetPartsInfoResponse message which is an answer to GetPartsInfo and contains positional
 *        information about the specified parts.
 * @date Created: 23 apr. 2013
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

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.data.Part;
import rexos.mas.data.Position;
import rexos.mas.data.ProductStep;
import rexos.mas.data.ScheduleData;
import rexos.mas.data.StepStatusCode;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceStep;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

/**
 * This behaviour handles the GetPartsInfoResponse message. This is an answer from the logistics agent for
 * the message GetPartsInfo. The answer will contain positional information about the specified parts. It will generate
 * a timeout after a specified period if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class GetPartsInfoResponse extends ReceiveOnceBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -7419729795873947786L;

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
	 * @var ProductStep productStep
	 *      The productStep from which the parts come.
	 */
	private ProductStep productStep;

	/**
	 * Creates a new GetPartsInfoResponse instance with the specified parameters. A default value of 2000 ms
	 * is used for the timeout.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 * @param productStep The productStep from which the parts come.
	 */
	public GetPartsInfoResponse(ServiceAgent agent, String conversationId, ProductStep productStep) {
		this(agent, 2000, conversationId, productStep);
	}

	/**
	 * Creates a new GetPartsInfoResponse instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 * @param millis the timeout period in milliseconds.
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 * @param productStep The productStep from which the parts come.
	 */
	public GetPartsInfoResponse(ServiceAgent agent, int millis, String conversationId, ProductStep productStep) {
		super(agent, millis, MessageTemplate.and(MessageTemplate.MatchConversationId(conversationId),
				MessageTemplate.MatchOntology("GetPartsInfoResponse")));
		this.agent = agent;
		this.conversationId = conversationId;
		this.productStep = productStep;
	}

	/**
	 * Handles an incoming message from the logisticsAgent. The message contains positional information about the
	 * specified parts. The message also contains the partId and partType of the output part of the productStep. Once a
	 * message is received the outputPart field of the productStep on the blackboard is filled in with the partType of
	 * the outputPart. Then updateParameters() is called on the service object mapped with the specified conversationId
	 * and the resulting serviceSteps are saved on the serviceStepsBlackBoard. Finally a FillPlaceholders message is
	 * send to the hardwareAgent to update the equipletSteps with the additional information. If a timeout occurs this
	 * method will call doDelete() on the service agent.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			try {
				ObjectId productStepId = productStep.getId();
				List<DBObject> dbServiceSteps =
						agent.getServiceStepBBClient().findDocuments(new BasicDBObject("productStepId", productStepId));
				ServiceStep[] serviceSteps = new ServiceStep[dbServiceSteps.size()];

				for(int i = 0; i < dbServiceSteps.size(); i++) {
					serviceSteps[i] = new ServiceStep((BasicDBObject) dbServiceSteps.get(i));
				}

				HashMap<Part, Position> parameters = (HashMap<Part, Position>) message.getContentObject();
				
				BasicDBList partList = new BasicDBList();
				
				for(Object part : parameters.keySet().toArray()){
					
					Part p = (Part)part;
					partList.add(p.toBasicDBObject());					
				}							
				
				agent.getProductStepBBClient().updateDocuments(new BasicDBObject("_id", productStep.getId()),
						new BasicDBObject("$set", new BasicDBObject("inputParts",partList)));
				
				Logger.log("%s got partsInfo: %s%n", agent.getLocalName(), parameters.toString());

				for(Entry<Part, Position> e : parameters.entrySet()) {
					if(e.getValue() == null) {
						agent.getProductStepBBClient()
								.updateDocuments(
										new BasicDBObject("_id", productStepId),
										new BasicDBObject("$set", new BasicDBObject("outputPart", e.getKey()
												.toBasicDBObject())));
						parameters.remove(e.getKey());
						break;
					}
				}
				ServiceStep[] parameterizedSteps =
						agent.GetServiceForConvId(conversationId).updateParameters(parameters,
								ServiceStep.sort(serviceSteps));

				ScheduleData scheduleData;
				int nextStartTime = productStep.getScheduleData().getStartTime();
				for(ServiceStep serviceStep : parameterizedSteps) {
					scheduleData = serviceStep.getScheduleData();
					scheduleData.setStartTime(nextStartTime);
					serviceStep.setScheduleData(scheduleData);
					serviceStep.setStatus(StepStatusCode.PLANNED);

					nextStartTime += scheduleData.getDuration();

					agent.getServiceStepBBClient().updateDocuments(
							new BasicDBObject("_id", serviceStep.getId()),
							new BasicDBObject("$set",
									new BasicDBObject("parameters", serviceStep.getParameters())
											.append("scheduleData", serviceStep.getScheduleData().toBasicDBObject())
											.append("status", serviceStep.getStatus().name())));
				}

				ACLMessage informMsg = new ACLMessage(ACLMessage.INFORM);
				informMsg.setOntology("FillPlaceholders");
				informMsg.setConversationId(message.getConversationId());
				informMsg.addReceiver(agent.getHardwareAgentAID());
				informMsg.setContentObject(parameterizedSteps[0].getId());
				agent.send(informMsg);

				agent.getProductStepBBClient().updateDocuments(new BasicDBObject("_id", productStep.getId()),
						new BasicDBObject("$set", new BasicDBObject("status", StepStatusCode.PLANNED.name())));
			} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException | IOException e) {
				Logger.log(e);
				agent.doDelete();
			}
		} else {
			Logger.log(agent.getName() + " - GetPartsInfoResponse timeout!");
			agent.doDelete();
		}
	}
}
