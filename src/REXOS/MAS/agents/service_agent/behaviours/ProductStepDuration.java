/**
 * @file rexos/mas/service_agent/behaviours/ProductStepDuration.java
 * @brief Handles the ProductStepDuration message which the equipletAgent sends to ask for the duration of a
 *        certain productStep.
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
import java.util.ArrayList;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.BehaviourCallbackItem;
import agents.data_classes.ParentBehaviourCallback;
import agents.data_classes.ProductStep;
import agents.service_agent.Service;
import agents.service_agent.ServiceAgent;
import agents.service_agent.ServiceStep;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the GetProductionStepDuration message. The equipletAgent sends this message to ask how long
 * execution of a certain productStep still will take.
 * 
 * @author Peter Bonnema
 * 
 */
public class ProductStepDuration extends ReceiveBehaviour implements ParentBehaviourCallback{
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -9066154686608658310L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate to match the messages.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("ProductStepDuration");
	
	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * Creates a new GetProductStepDuration instance with the specified parameters.
	 * 
	 * @param serviceAgent the agent this behaviour belongs to.
	 */
	public ProductStepDuration(ServiceAgent serviceAgent) {
		super(serviceAgent, MESSAGE_TEMPLATE);
		this.serviceAgent = serviceAgent;
	}

	/**
	 * Handles an incoming message from the equipletAgent. The equipletAgent sends this message to ask for the duration
	 * of the serviceStep and all serviceSteps linked to it specified by the ObjectId in the message. First the service
	 * object generated in CanDoProductStep is retrieved from the serviceAgent to generate serviceSteps. The generated
	 * serviceSteps are stored on the serviceStepBlackBoard and a GetServiceStepDuration message is send to the
	 * hardwareAgent. Finally a GetServiceStepsDurationResponse behaviour is started to handle the answer.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		try {
			ObjectId productStepId = (ObjectId) message.getContentObject();
			ProductStep productStep =
					new ProductStep((BasicDBObject) serviceAgent.getProductStepBBClient().findDocumentById(productStepId));
			int productStepType = productStep.getType();

			Logger.log(LogLevel.DEBUG, "%s got message GetProductStepDuration for step type %s%n", serviceAgent.getLocalName(),
					productStepType);

			Service service = serviceAgent.getServiceForConvId(message.getConversationId());
			BasicDBObject parameters = productStep.getParameters();
			ServiceStep[] serviceSteps = service.getServiceSteps(productStepType, parameters);
			for(ServiceStep serviceStep : serviceSteps) {
				serviceStep.setProductStepId(productStepId);
			}

			Logger.log(LogLevel.DEBUG, "%s asking %s for duration of %d steps%n", serviceAgent.getLocalName(), serviceAgent.getHardwareAgentAID()
					.getLocalName(), serviceSteps.length);
			ObjectId[]serviceStepIds = new ObjectId[serviceSteps.length];
			ObjectId serviceStepId = null;
			BlackboardClient serviceStepBB = serviceAgent.getServiceStepBBClient();
			//link all the service steps to eachother
			for(int i = serviceSteps.length - 1; i >= 0; i--) {
				serviceSteps[i].setNextServiceStep(serviceStepId);
				serviceStepId = serviceStepBB.insertDocument(serviceSteps[i].toBasicDBObject());
				serviceStepIds[i] = serviceStepId;
			}

			serviceAgent.addBehaviour(new ServiceStepDuration(serviceAgent, this, message.getConversationId(), serviceStepIds	));
			
	//		ACLMessage askMessage = new ACLMessage(ACLMessage.QUERY_IF);
	//		askMessage.addReceiver(serviceAgent.getHardwareAgentAID());
	//		askMessage.setOntology("ServiceStepDuration");
	//		askMessage.setConversationId(message.getConversationId());
	//		askMessage.setContentObject(serviceStepId);
	//		serviceAgent.send(askMessage);
		} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, e);
			serviceAgent.doDelete();
		}
	}

	@Override
	public void callback(ACLMessage result, BehaviourCallbackItem arguments) {
		
		long duration = (long) arguments.getArgument("duration");
		
		ACLMessage answer = new ACLMessage(ACLMessage.INFORM);
		answer.addReceiver(serviceAgent.getEquipletAgentAID());
		answer.setConversationId(result.getConversationId());
		try{
			answer.setContentObject(duration);
		}catch (IOException e){
			Logger.log(LogLevel.DEBUG, duration);
			serviceAgent.doDelete();
		}
		answer.setOntology("ProductStepDuration");
		serviceAgent.send(answer);
		
	}
}
