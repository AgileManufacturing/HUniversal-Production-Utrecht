/**
 * @file rexos/mas/service_agent/behaviours/ScheduleStep.java
 * @brief Handles the ScheduleStep message which the equipletAgent sends to let the serviceAgent schedule the
 *        serviceSteps.
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

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import libraries.blackboard_client.GeneralMongoException;
import libraries.blackboard_client.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data.ProductStep;
import agents.service_agent.ServiceAgent;
import agents.shared_behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the ScheduleStep message. The equipletAgent sends this message to ask the serviceAgent to
 * schedule the serviceSteps of the specified productStep.
 * 
 * @author Peter Bonnema
 * 
 */
public class ScheduleStep extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -346155137703956948L;

	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent agent;

	/**
	 * Creates a new ScheduleStep instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 */
	public ScheduleStep(Agent agent) {
		super(agent, MessageTemplate.MatchOntology("ScheduleStep"));
		this.agent = (ServiceAgent) agent;
	}

	/**
	 * Handles an incoming message from the equipletAgent. The equipletAgent sends this message to ask the serviceAgent
	 * to schedule a productStep. This includes updating the parameters and adding schedule data to all serviceSteps
	 * that were generated from the productStep specified in the message. To update the parameters additional
	 * information is required from the logisticsAgent so first a conversation is initiated by sending a
	 * ArePartsAvailable message. Also a behaviour is started to handle the answer. This causes a string of messages to
	 * be send and received and in the end all serviceSteps are scheduled.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			try {
				Logger.log(LogLevel.DEBUG, "%s scheduling step with Logistics%n", agent.getLocalName());

				ProductStep productStep =
						new ProductStep((BasicDBObject) agent.getProductStepBBClient().findDocumentById(
								(ObjectId) message.getContentObject()));
				
				agent.mapConvIdWithProductStepId(message.getConversationId(), productStep.getId());

				ACLMessage sendMsg = new ACLMessage(ACLMessage.QUERY_IF);
				sendMsg.setConversationId(message.getConversationId());
				sendMsg.addReceiver(agent.getLogisticsAID());
				sendMsg.setOntology("ArePartsAvailable");
				sendMsg.setContentObject(productStep);
				agent.send(sendMsg);
			} catch(InvalidDBNamespaceException | GeneralMongoException | UnreadableException | IOException e) {
				Logger.log(LogLevel.ERROR, e);
				agent.doDelete();
			}
		}
	}
}
