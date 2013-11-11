/**
 * @file rexos/mas/service_agent/behaviours/ServiceStepDuration.java
 * @brief Handles the ServiceStepDuration message from the hardwareAgent that indicates that the duration of
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
package agents.service_agent.behaviours;

import java.io.IOException;
import java.io.Serializable;
import java.util.HashMap;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.service_agent.ServiceAgent;
import agents.service_agent.ServiceStep;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the ServiceStepDuration message. The hardwareAgent sends this message in response
 * to the ServiceStepDuration message to indicate that it saved the duration of all serviceSteps on the
 * serviceStepBlackboard. It will generate a timeout after a specified period if no message is received.
 * 
 * @author Peter
 * 
 */
public class ServiceStepDuration extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -3454535337099894852L;

	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * @var ParentBehaviourCallback parentBehaviourCallback
	 * 		The parentbehaviour callback this redirect calls back to
	 */
	private ParentBehaviourCallback parentBehaviourCallback;

	/**
	 * @var ObjectId objectId
	 * 		The ObjectId used for the message
	 */
	private ObjectId[] serviceStepIds;

	/**
	 * @var String conversationId
	 * 		The conversationId used for the message
	 */
	private String conversationId;

	/**
	 * Creates a new ServiceStepDuration instance with the specified parameters.
	 * 
	 * @param serviceAgent the service agent this behaviour belongs to.
	 * @param parentBehaviourCallback the parentbehaviour this behaviour calls back to 
	 * @param conversationId the conversationId that any messages sent or received by this behaviour will have.
	 * @param serviceStepIds The service step ids used to check the duration
	 */
	public ServiceStepDuration(ServiceAgent serviceAgent, ParentBehaviourCallback parentBehaviourCallback,
			String conversationId, ObjectId[] serviceStepIds) {
		super(serviceAgent, MessageTemplate.MatchOntology("ServiceStepDuration"));
		this.serviceAgent = serviceAgent;
		
		this.parentBehaviourCallback = parentBehaviourCallback;
		
		this.serviceStepIds = serviceStepIds;
		this.conversationId = conversationId;
	}

	@Override
	public void onStart(){
		ACLMessage responseMessage = new ACLMessage(ACLMessage.QUERY_REF);
		responseMessage.addReceiver(serviceAgent.getHardwareAgentAID());
		responseMessage.setConversationId(conversationId);
		if (serviceStepIds != null){
			try {
				responseMessage.setContentObject(serviceStepIds);
			} catch (IOException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		}
		responseMessage.setOntology("ServiceStepDuration");
		serviceAgent.send(responseMessage);
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
				HashMap<ObjectId, Long> serviceStepDurations = (HashMap<ObjectId, Long>) message.getContentObject();
				//ObjectId nextStep = (ObjectId) message.getContentObject();
				long duration = 0;
				for ( ObjectId serviceStep: serviceStepDurations.keySet()){
					duration += serviceStepDurations.get(serviceStep);
				}
			//	while(nextStep != null) {
			//		serviceStep.fromBasicDBObject((BasicDBObject) serviceAgent.getServiceStepBBClient().findDocumentById(nextStep));
			//		duration += serviceStep.getScheduleData().getDuration();
			//		nextStep = serviceStep.getNextServiceStep();
			//	}

		//		ObjectId productStepId = serviceStep.getProductStepId();
		//		ProductStep productStep =
		//				new ProductStep((BasicDBObject) serviceAgent.getProductStepBBClient().findDocumentById(productStepId));
		//		ScheduleData scheduleData = productStep.getScheduleData();
		//		scheduleData.setDuration(duration);

		//		Logger.log(LogLevel.DEBUG, "Saving duration of %d in prod. step %s%n", duration, productStepId);
		//		serviceAgent.getProductStepBBClient().updateDocuments(new BasicDBObject("_id", productStepId),
		//				new BasicDBObject("$set", new BasicDBObject("scheduleData", scheduleData.toBasicDBObject())));
				
				BehaviourCallbackItem arguments = new BehaviourCallbackItem();
				arguments.addArgument("duration", duration);
				parentBehaviourCallback.callback(message, arguments);
				serviceAgent.removeBehaviour(this);
				
			} catch(UnreadableException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		} else {
			Logger.log(LogLevel.DEBUG, serviceAgent.getName() + " - GetServiceStepDurationResponse timeout!");
			serviceAgent.doDelete();
		}
	}
}
