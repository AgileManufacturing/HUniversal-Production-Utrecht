/**
 * @file CheckForModulesResponse.java
 * @brief 
 * @date Created: 18 apr. 2013
 *
 * @author Peter Bonnema
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/
package rexos.mas.serviceAgent.behaviour;

import rexos.mas.serviceAgent.ServiceAgent;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import rexos.mas.behaviours.ReceiveOnceBehaviour;

/**
 * @author Peter
 * 
 */
public class CheckForModulesResponse extends ReceiveOnceBehaviour {
	private static final long serialVersionUID = 1L;
	private ServiceAgent agent;

	/**
	 * @param a
	 */
	public CheckForModulesResponse(Agent a) {
		this(a, 2000);
	}

	/**
	 * @param a
	 * @param millis
	 */
	public CheckForModulesResponse(Agent a, int millis) {
		super(a, millis, MessageTemplate
				.MatchOntology("CheckForModulesResponse"));
		agent = (ServiceAgent) a;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			ACLMessage reply = message.createReply();
			reply.clearAllReceiver();
			reply.addReceiver(agent.getEquipletAgentAID());
			reply.setPerformative(message.getPerformative());
			reply.setOntology("CanDoProductionStepResponse");
			getAgent().send(reply);
			System.out.format("%s sending step availability (%b)%n", getAgent()
					.getLocalName(), message.getPerformative() == ACLMessage.CONFIRM);
		} else {
			// TODO handle timeout
		}
	}
}
