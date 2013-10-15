/**
 * @file rexos/mas/service_agent/behaviours/CanDoProductStep.java
 * @brief Handles the CanDoProductStep message that the equipletAgent sends to ask whether the equiplet is able to
 *        perform a certain productStep.
 * @date Created: 20 apr. 2013
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
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the CanDoProductStep message. The equipletAgent sends this message to ask whether the equiplet
 * is able to perform a certain productStep. It will generate a timeout after 2 seconds if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class CanPerformProductionStep extends ReceiveBehaviour implements ParentBehaviourCallback{
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * Creates a new CanDoProductStep instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 */
	public CanPerformProductionStep(ServiceAgent agent) {
		super(agent, MessageTemplate.MatchOntology("CanPerformProductionStep"));
		this.serviceAgent = agent;
	}

	/**
	 * Handles an incoming message from the equipletAgent. With this message the equipletAgent asks whether the
	 * productStep specified by its ObjectId in the message can be performed by the equiplet.
	 * First, a service is generated for the stepType of this productStep. Then a mapping of this service object with
	 * the conversationId of the message is added in the serviceAgent. Finally a CheckForModules message is send to the
	 * hardwareAgent to ask whether all required modules for the service are present and a behaviour is started to
	 * handle the answer.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		
		if ( message.getPerformative() == ACLMessage.QUERY_IF){
			try {
				ObjectId productStepId = (ObjectId) message.getContentObject();
				ProductStep productStep =
						new ProductStep((BasicDBObject) serviceAgent.getProductStepBBClient().findDocumentById(productStepId));
				int stepType = productStep.getType();
				BasicDBObject parameters = productStep.getParameters();
	
				//Logger.log("%s got message CanDoProductStep for step type %s%n", agent.getLocalName(), stepType);
	
				Service[] services = serviceAgent.getServiceFactory().getServicesForStep(stepType);
	
				ArrayList<Service> possibleServices = new ArrayList<Service>();
				for(Service service : services) {
					if(service.canDoStep(stepType, parameters)) {
						possibleServices.add(service);
					}
				}
	
				if(!possibleServices.isEmpty()) {
					// TODO (out of scope)implement algorithm to intelligently choose a service here
					Service chosenService = possibleServices.get(0);
	
					serviceAgent.mapConvIdWithService(message.getConversationId(), chosenService);
	
					serviceAgent.addBehaviour(new RequiredModulesPresent(serviceAgent, this, message.getConversationId(), 
																		chosenService.getModuleGroupIds(stepType, parameters) ));
			//		ACLMessage queryIfHWA = new ACLMessage(ACLMessage.QUERY_IF);
			//		queryIfHWA.setConversationId(message.getConversationId());
			//		queryIfHWA.addReceiver(serviceAgent.getHardwareAgentAID());
			//		queryIfHWA.setOntology("RequiredModulesPresent");
			//		queryIfHWA.setContentObject(chosenService.getModuleGroupIds(stepType, parameters));
			//		serviceAgent.send(queryIfHWA);
				} else {
					ACLMessage disconfirmReply = message.createReply();
					disconfirmReply.setPerformative(ACLMessage.DISCONFIRM);
					disconfirmReply.setOntology("CanPerformProductionStep");
					getAgent().send(disconfirmReply);
					Logger.log(LogLevel.DEBUG, "%s sending step availability (%b)%n", getAgent().getLocalName(),
							disconfirmReply.getPerformative() == ACLMessage.CONFIRM);
				}
			} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, e);
			}
		}
	}

	@Override
	public void callback(ACLMessage result, BehaviourCallbackItem arguments) {
		ACLMessage reply = new ACLMessage(result.getPerformative());
		reply.setConversationId(result.getConversationId());
		reply.addReceiver(serviceAgent.getEquipletAgentAID());
		reply.setOntology("CanPerformProductionStep");
		serviceAgent.send(reply);
	}
}
