/**
 * @file rexos/mas/service_agent/behaviours/GetProductStepDuration.java
 * @brief Handles the GetProductionStepDuration message which the equipletAgent sends to ask for the duration of a
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
package rexos.mas.service_agent.behaviours;

import java.io.IOException;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ProductStep;
import rexos.mas.service_agent.Service;
import rexos.mas.service_agent.ServiceAgent;
import rexos.mas.service_agent.ServiceStep;
import rexos.utillities.log.LogLevel;
import rexos.utillities.log.Logger;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the GetProductionStepDuration message. The equipletAgent sends this message to ask how long
 * execution of a certain productStep still will take.
 * 
 * @author Peter Bonnema
 * 
 */
public class GetProductStepDuration extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -9066154686608658310L;

	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent agent;

	/**
	 * Creates a new GetProductStepDuration instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 */
	public GetProductStepDuration(ServiceAgent agent) {
		super(agent, MessageTemplate.MatchOntology("GetProductionStepDuration"));
		this.agent = agent;
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
					new ProductStep((BasicDBObject) agent.getProductStepBBClient().findDocumentById(productStepId));
			int productStepType = productStep.getType();

			//Logger.log("%s got message GetProductStepDuration for step type %s%n", agent.getLocalName(),
					//productStepType);

			Service service = agent.getServiceForConvId(message.getConversationId());
			BasicDBObject parameters = productStep.getParameters();
			ServiceStep[] serviceSteps = service.getServiceSteps(productStepType, parameters);
			for(ServiceStep serviceStep : serviceSteps) {
				serviceStep.setProductStepId(productStepId);
			}

			Logger.log(LogLevel.DEBUG, "%s asking %s for duration of %d steps%n", agent.getLocalName(), agent.getHardwareAgentAID()
					.getLocalName(), serviceSteps.length);

			ObjectId serviceStepId = null;
			BlackboardClient serviceStepBB = agent.getServiceStepBBClient();
			for(int i = serviceSteps.length - 1; i >= 0; i--) {
				serviceSteps[i].setNextStep(serviceStepId);
				serviceStepId = serviceStepBB.insertDocument(serviceSteps[i].toBasicDBObject());
			}

			ACLMessage askMessage = new ACLMessage(ACLMessage.QUERY_IF);
			askMessage.addReceiver(agent.getHardwareAgentAID());
			askMessage.setOntology("GetServiceStepDuration");
			askMessage.setConversationId(message.getConversationId());
			askMessage.setContentObject(serviceStepId);
			agent.send(askMessage);
		} catch(UnreadableException | InvalidDBNamespaceException | GeneralMongoException | IOException e) {
			Logger.log(LogLevel.ERROR, e);
			agent.doDelete();
		}
	}
}
