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
package rexos.mas.service_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.equiplet_agent.ProductStep;
import rexos.mas.service_agent.Service;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceFactory;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the CanDoProductStep message. The equipletAgent sends this message to ask whether the equiplet
 * is able to perform a certain productStep. It will generate a timeout after 2 seconds if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class CanDoProductStep extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent agent;

	/**
	 * @var ServiceFactory factory
	 *      The serviceFactory to create a service with that will handle the translation of the productStep to
	 *      serviceSteps.
	 */
	private ServiceFactory factory;

	/**
	 * Creates a new CanDoProductStep instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 * @param factory the serviceFactory to create a service for this productStep with.
	 */
	public CanDoProductStep(ServiceAgent agent, ServiceFactory factory) {
		super(agent, MessageTemplate.MatchOntology("CanDoProductionStep"));
		this.agent = agent;
		this.factory = factory;
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
		try {
			ObjectId productStepId = (ObjectId) message.getContentObject();
			ProductStep productStep =
					new ProductStep((BasicDBObject) agent.getProductStepBBClient().findDocumentById(productStepId));
			int stepType = productStep.getType();
			BasicDBObject parameters = productStep.getParameters();

			Logger.log("%s got message CanDoProductionStep for step type %s%n", agent.getLocalName(), stepType);

			Service[] services = factory.getServicesForStep(stepType);
			if(services.length > 0) {
				Service service = services[0];
				agent.MapConvIdWithService(message.getConversationId(), service);

				ACLMessage msg = new ACLMessage(ACLMessage.QUERY_IF);
				msg.setConversationId(message.getConversationId());
				msg.addReceiver(agent.getHardwareAgentAID());
				msg.setOntology("CheckForModules");
				msg.setContentObject(service.getModuleIds(stepType, parameters));
				agent.send(msg);

				agent.addBehaviour(new CheckForModulesResponse(agent));
			} else {
				ACLMessage reply = message.createReply();
				reply.setPerformative(ACLMessage.DISCONFIRM);
				reply.setOntology("CanDoProductionStepResponse");
				getAgent().send(reply);
				Logger.log("%s sending step availability (%b)%n", getAgent().getLocalName(),
						reply.getPerformative() == ACLMessage.CONFIRM);
			}
		} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException | IOException e) {
			Logger.log(e);
			agent.doDelete();
		}
	}
}
