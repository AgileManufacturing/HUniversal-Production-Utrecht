/**
 * @file rexos/mas/service_agent/behaviours/ArePartsAvailableResponse.java
 * @brief Handles the ArePartsAvailableResponse message which is an answer to ArePartsAvailable and indicates
 *        whether the specified parts are present and available in general.
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

import java.io.IOException;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.LogLevel;
import rexos.mas.data.ProductStep;
import rexos.mas.data.StepStatusCode;
import rexos.mas.service_agent.ServiceAgent;

import com.mongodb.BasicDBObject;

/**
 * This behaviour handles the ArePartsAvailableResponse message. This is an answer from the logistics agent for
 * the message ArePartsAvailable. The answer will indicate whether the specified parts are present in the grid and
 * available in general. It will generate a timeout after a specified period if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class ArePartsAvailableResponse extends ReceiveBehaviour {
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
	 * Creates a new ArePartsAvailableInTimeResponse instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 */
	public ArePartsAvailableResponse(ServiceAgent agent) {
		super(agent, MessageTemplate.MatchOntology("ArePartsAvailableResponse"));
		this.agent = agent;
	}

	/**
	 * Handles an incoming message from the logistics agent. The message confirms or disconfirms whether all the
	 * specified parts are present in the grid and available in general. Once a message is received a
	 * ArePartsAvailableInTime message is send to ask the logistics agent whether the specified parts will be available
	 * in time for the execution of the productStep. If a timeout occurs this method will call doDelete() on the service
	 * agent.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			Logger.logAclMessage(message, 'r');
			try {
				Logger.log(LogLevel.DEBUG, "%s ArePartsAvailableResponse%n", agent.getLocalName());
				
				BlackboardClient productStepBBClient = agent.getProductStepBBClient();
				ObjectId productStepId = agent.getProductStepIdForConvId(message.getConversationId());
				ProductStep productStep = new ProductStep((BasicDBObject) productStepBBClient.findDocumentById(productStepId));
				if(message.getPerformative() == ACLMessage.CONFIRM) {
					ACLMessage sendMsg = message.createReply();
					sendMsg.setOntology("ArePartsAvailableInTime");
					sendMsg.setPerformative(ACLMessage.QUERY_IF);
					sendMsg.setContentObject(productStep.getInputParts());
					agent.send(sendMsg);
					Logger.logAclMessage(sendMsg, 's');
				} else {
					productStepBBClient.updateDocuments(
							new BasicDBObject("_id", productStep.getId()),
							new BasicDBObject("$set", new BasicDBObject("status", StepStatusCode.ABORTED.name())
									.append("statusData", new BasicDBObject("reason", "missing productStep"))));
				}
			} catch(IOException | InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, e);
				agent.doDelete();
			}
		} else {
			Logger.log(LogLevel.WARNING, agent.getName() + " - ArePartsAvailableResponse timeout!");
			agent.doDelete();
		}
	}
}
