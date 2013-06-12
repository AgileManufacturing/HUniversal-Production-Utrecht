/**
 * @file rexos/mas/service_agent/behaviours/CheckForModulesResponse.java
 * @brief Handles the CheckForModulesResponse message from the hardwareAgent that indicates whether all required modules
 *        for a service are present.
 * @date Created: 18 apr. 2013
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
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.service_agent.ServiceAgent;

/**
 * This behaviour handles the CheckForModulesResponse message. The hardwareAgent sends this message in response to the
 * CheckForModules message to indicate whether all required modules for a service are present. It will generate a
 * timeout after a specified period if no message is received.
 * 
 * @author Peter Bonnema
 * 
 */
public class CheckForModulesResponse extends ReceiveOnceBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID of this class.
	 */
	private static final long serialVersionUID = -7514590874724120963L;
	
	/**
	 * @var ServiceAgent agent
	 *      The service agent this behaviour belongs to.
	 */
	private ServiceAgent agent;

	/**
	 * Creates a new ArePartsAvailableResponse instance with the specified parameters. A default value of 2000 ms is
	 * used for the timeout.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 */
	public CheckForModulesResponse(ServiceAgent agent) {
		this(agent, 2000);
	}

	/**
	 * Creates a new ArePartsAvailableResponse instance with the specified parameters.
	 * 
	 * @param agent the agent this behaviour belongs to.
	 * @param millis the timeout period in milliseconds.
	 */
	public CheckForModulesResponse(ServiceAgent agent, int millis) {
		super(agent, millis, MessageTemplate.MatchOntology("CheckForModulesResponse"));
		this.agent = agent;
	}

	/**
	 * Handles an incoming message from the hardwareAgent. The message confirms or disconfirms whether all modules
	 * required for a service are present. Once a message is received a CanDoProductionStepResponse message is send to
	 * the equipletAgent as an answer to the previously received CanDoProductionStep message. If a timeout occurs this
	 * method will call doDelete() on the service agent.
	 * 
	 * @param message the message to handle or null on timeout.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			ACLMessage reply = message.createReply();
			reply.clearAllReceiver();
			reply.addReceiver(agent.getEquipletAgentAID());
			reply.setPerformative(message.getPerformative());
			reply.setOntology("CanDoProductionStepResponse");
			getAgent().send(reply);
			Logger.log("%s sending step availability (%b)%n", getAgent().getLocalName(),
					message.getPerformative() == ACLMessage.CONFIRM);
		} else {
			Logger.log(agent.getName() + " - CheckForModulesResponse timeout!");
			agent.doDelete();
		}
	}
}
