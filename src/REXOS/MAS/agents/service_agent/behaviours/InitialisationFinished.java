/**
 * @file rexos/mas/service_agent/behaviours/InitialisationFinished.java
 * @brief Handles the InitialisationFinished message which the hardwareAgent sends to indicate its initialized.
 * @date Created: 2013-04-02
 * 
 * @author Hessel Meulenbeld
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
 **/
package agents.service_agent.behaviours;

import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.service_agent.ServiceAgent;
import agents.shared_behaviours.ReceiveBehaviour;

/**
 * This behaviour handles the InitialisationFinished message. The hardwareAgent sends this message to indicate its done
 * initializing.
 * 
 * @author Hessel Meulenbeld
 * 
 */
public class InitialisationFinished extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 581780075377109392L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate this behaviour listens to. This behaviour listens to the ontology:
	 *      InitialisationFinished.
	 */
	private static MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("InitialisationFinished");

	/**
	 * @var ServiceAgent serviceAgent
	 *      The serviceAgent related to this behaviour.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * Instantiates a new InitialisationFinished.
	 * 
	 * @param serviceAgent The agent for this behaviour
	 */
	public InitialisationFinished(ServiceAgent serviceAgent) {
		super(serviceAgent, MESSAGE_TEMPLATE);
		this.serviceAgent = serviceAgent;
	}

	/**
	 * Handles an incoming message from the hardwareAgent. The hardwareAgent sends this message to indicate that it's
	 * completely initialized. Then the same message is send to the equipletAgent to indicate the same.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			Logger.log(LogLevel.DEBUG, "%s received message from %s%n", myAgent.getLocalName(), message.getSender().getLocalName(),
					message.getOntology());

			ACLMessage response = new ACLMessage(ACLMessage.CONFIRM);
			response.addReceiver(serviceAgent.getEquipletAgentAID());
			response.setOntology("InitialisationFinished");
			serviceAgent.send(response);
		} else {
			Logger.log(LogLevel.DEBUG, serviceAgent.getName() + " - InitialisationFinished timeout!");
			serviceAgent.doDelete();
		}
	}
}
