/**
 * @file rexos/mas/hardware_agent/behaviours/ServiceAgentDied.java
 * @brief Behaviour for handling the messages with the ontology ServiceAgentDied
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
package agents.hardware_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import agents.hardware_agent.HardwareAgent;
import agents.shared_behaviours.ReceiveBehaviour;

/** The Class ServiceAgentDied. */
public class ServiceAgentDied extends ReceiveBehaviour {
	/**
	 * @var static final long serialVersionUID
	 *      The serial version UID for this class
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate this behaviour listens to. This behaviour
	 *      listens to the ontology: ServiceAgentDied.
	 */
	private static MessageTemplate MESSAGE_TEMPLATE = MessageTemplate
			.MatchOntology("ServiceAgentDied");

	/**
	 * @var HardwareAgent hardwareAgent
	 *      The hardwareAgent related to this behaviour.
	 */
	private HardwareAgent hardwareAgent;

	/**
	 * Instantiates a new service agent died.
	 * 
	 * @param a
	 *            The agent for this behaviour
	 */
	public ServiceAgentDied(HardwareAgent hardwareAgent) {
		super(hardwareAgent, MESSAGE_TEMPLATE);
		this.hardwareAgent = hardwareAgent;
	}

	/**
	 * Function to handle the incoming messages for this behaviour. Handles the
	 * response when the service agent dies.
	 * 
	 * @param message
	 *            The received message.
	 */
	@Override
	public void handle(ACLMessage message) {
		//remove the hardwareAgent
		hardwareAgent.doDelete();
	}
}
