/**
 * @file InitialisationFinished.java
 * @brief Behaviour for handling the messages with the ontology InitialisationFinished
 * @date Created: 2013-04-02
 * 
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 **/
package rexos.mas.service_agent.behaviour;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.service_agent.ServiceAgent;

/** The Class InitialisationFinished. */
public class InitialisationFinished extends ReceiveOnceBehaviour {
	/**
	 * @var static final long serialVersionUID The serial version UID for this
	 *      class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate messageTemplate The messageTemplate this behaviour
	 *      listens to. This behaviour listens to the ontology: InitialisationFinished.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate
			.MatchOntology("InitialisationFinished");

	/**
	 * @var ServiceAgent serviceAgent The serviceAgent related to this
	 *      behaviour.
	 */
	private ServiceAgent serviceAgent;

	/**
	 * Instantiates a new can perform step.
	 * 
	 * @param a The agent for this behaviour
	 */
	public InitialisationFinished(Agent a) {
		super(a, 2000, messageTemplate);
		serviceAgent = (ServiceAgent) a;
	}

	/**
	 * Function to handle the incoming messages for this behaviour. Handles the
	 * response to the InitialisationFinished.
	 * 
	 * @param message The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		if(message != null) {
			System.out.format("%s received message from %s%n", myAgent.getLocalName(), message
					.getSender().getLocalName(), message.getOntology());
			
			ACLMessage response = new ACLMessage(ACLMessage.CONFIRM);
			response.addReceiver(serviceAgent.getEquipletAgentAID());
			response.setOntology("InitialisationFinished");
			serviceAgent.send(response);
		} else {
			//TODO handle timeout
			System.out.println(serviceAgent.getName() + " - InitialisationFinished timeout!");
			serviceAgent.doDelete();
		}
	}
}
