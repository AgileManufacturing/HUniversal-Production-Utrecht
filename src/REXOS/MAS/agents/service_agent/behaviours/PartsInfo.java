/**
 * @file rexos/mas/service_agent/behaviours/PartsInfo.java
 * @brief Handles the PartsInfo message which is an answer to PartsInfo and contains positional
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
package agents.service_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.Part;
import agents.data_classes.Position;
import agents.data_classes.ProductStep;
import agents.data_classes.ScheduleData;
import agents.data_classes.StepStatusCode;
import agents.service_agent.ServiceAgent;
import agents.service_agent.ServiceStep;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

/**
 * This behaviour handles the PartsInfo message. This is an answer from the logistics agent for
 * the message PartsInfo. The answer will contain positional information about the specified parts. It will generate
 * a timeout after a specified period if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class PartsInfo extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -7419729795873947786L;
	
	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate to match the messages.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("PartsInfo");
	
	/**
	 * @var ServiceAgent serviceAgent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * @var ParentBehaviourCallback parentBehaviourCallback
	 * 		The parentbehaviour callback this redirect calls back to
	 */
	private ParentBehaviourCallback parentBehaviourCallback;
	
	/**
	 * @var ProductStep productStep
	 * 		The ProductStep object used for the message
	 */
	private ProductStep productStep;

	/**
	 * @var String conversationId
	 * 		The conversationId used for the message
	 */
	private String conversationID;

	/**
	 * Creates a new GetPartsInfoResponse instance with the specified parameters.
	 * 
	 * @param serviceAgent the serviceAgent this behaviour belongs to.
	 * @param parentBehaviourCallback the behaviour callback to.
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 * @param productStep The productStep from which the parts come.
	 */
	public PartsInfo(ServiceAgent serviceAgent, ParentBehaviourCallback parentBehaviourCallback,
			String conversationID, ProductStep productStep) {
		super(serviceAgent, MESSAGE_TEMPLATE);
		this.serviceAgent = serviceAgent;
		
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.productStep = productStep;
		this.conversationID = conversationID;
	}

	@Override
	public void onStart(){
		ACLMessage responseMessage = new ACLMessage(ACLMessage.QUERY_REF);
		responseMessage.addReceiver(serviceAgent.getLogisticsAID());
		responseMessage.setConversationId(conversationID);
		if (productStep != null){
			try {
				responseMessage.setContentObject(productStep.getInputParts());
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, "", e);
			}
		}
		responseMessage.setOntology("PartsInfo");
		serviceAgent.send(responseMessage);
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
				BlackboardClient productStepBBClient = serviceAgent.getProductStepBBClient();
				BlackboardClient serviceStepBBClient = serviceAgent.getServiceStepBBClient();

				String conversationId = message.getConversationId();
				ObjectId productStepId = serviceAgent.getProductStepIdForConvId(conversationId);
				serviceAgent.removeConvIdProductStepIdMapping(conversationId);
				List<DBObject> dbServiceSteps =
						serviceStepBBClient.findDocuments(new BasicDBObject("productStepId", productStepId));
				ServiceStep[] serviceSteps = new ServiceStep[dbServiceSteps.size()];

				for(int i = 0; i < dbServiceSteps.size(); i++) {
					serviceSteps[i] = new ServiceStep((BasicDBObject) dbServiceSteps.get(i));
				}

				HashMap<Part, Position> partParameters = (HashMap<Part, Position>) message.getContentObject();

				BasicDBList partList = new BasicDBList();
				for(Object part : partParameters.keySet().toArray()) {
					Part p = (Part) part;
					partList.add(p.toBasicDBObject());
				}

				productStepBBClient.updateDocuments(new BasicDBObject("_id", productStepId), new BasicDBObject("$set",
						new BasicDBObject("inputParts", partList)));

				Logger.log(LogLevel.DEBUG, "%s got partsInfo: %s%n", serviceAgent.getLocalName(), partParameters.toString());

				for(Entry<Part, Position> e : partParameters.entrySet()) {
					if(e.getValue() == null) {
						productStepBBClient.updateDocuments(new BasicDBObject("_id", productStepId), new BasicDBObject(
								"$set", new BasicDBObject("outputPart", e.getKey().toBasicDBObject())));
						partParameters.remove(e.getKey());
						break;
					}
				}
				ServiceStep[] parameterizedSteps =
						serviceAgent.getServiceForConvId(conversationId).updateParameters(partParameters,
								ServiceStep.sort(serviceSteps));
				
				serviceAgent.removeConvIdServiceMapping(conversationId);

				ScheduleData scheduleData;
				ProductStep productStep =
						new ProductStep((BasicDBObject) productStepBBClient.findDocumentById(productStepId));
				long nextStartTime = productStep.getScheduleData().getStartTime();
				for(ServiceStep serviceStep : parameterizedSteps) {
					scheduleData = serviceStep.getScheduleData();
					scheduleData.setStartTime(nextStartTime);
					serviceStep.setScheduleData(scheduleData);
					serviceStep.setServiceStepStatus(StepStatusCode.PLANNED);

					nextStartTime += scheduleData.getDuration();

					serviceStepBBClient.updateDocuments(
							new BasicDBObject("_id", serviceStep.getId()),
							new BasicDBObject("$set", new BasicDBObject("parameters", serviceStep.getParameters())
									.append("scheduleData", serviceStep.getScheduleData().toBasicDBObject()).append(
											"status", serviceStep.getServiceStepStatus().name())));
				}
				
				BehaviourCallbackItem arguments = new BehaviourCallbackItem();
				arguments.addArgument("stepId", parameterizedSteps[0].getId());
				parentBehaviourCallback.callback(message, arguments);
				
				
				productStepBBClient.updateDocuments(new BasicDBObject("_id", productStepId), new BasicDBObject("$set",
						new BasicDBObject("status", StepStatusCode.PLANNED.name())));
			} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, "", e);
				serviceAgent.doDelete();
			}
			serviceAgent.removeBehaviour(this);
		} else {
			Logger.log(LogLevel.DEBUG, "" + serviceAgent.getName() + " - GetPartsInfoResponse timeout!");
			serviceAgent.doDelete();
		}
	}
}
