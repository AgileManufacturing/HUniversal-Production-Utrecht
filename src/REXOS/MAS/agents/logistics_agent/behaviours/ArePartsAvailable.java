/**
 * @file rexos/mas/logistics_agent/behaviours/ArePartsAvailable.java
 * @brief Responds to questions whether or not a specific part is available.
 * @date Created: 20 apr. 2013
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
package agents.logistics_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data.Part;
import agents.data.ProductStep;
import agents.shared_behaviours.ReceiveBehaviour;

/**
 * Responds to questions whether or not a specific part is available.
 */
public class ArePartsAvailable extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * Constructs the behaviour for the given agent.
	 * @param a The agent associated with this behaviour.
	 */
	public ArePartsAvailable(Agent a) {
		super(a, MessageTemplate.MatchOntology("ArePartsAvailable"));
	}

	/**
	 * 
	 * @see
	 * rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		try {
			Logger.log(LogLevel.DEBUG, "ArePartsAvailable%n", 0, myAgent.getLocalName());
			
			Part[] parts = ((ProductStep) message.getContentObject()).getInputParts();
			ACLMessage reply = message.createReply();
			reply.setOntology("ArePartsAvailableResponse");
			//TODO (out of scope) determine actual part availability 
			reply.setPerformative(ACLMessage.CONFIRM);
			myAgent.send(reply);

			myAgent.addBehaviour(new ArePartsAvailableInTime(myAgent, message.getConversationId()));
			Logger.log(LogLevel.DEBUG, "PartTypes { %s } are available%n", 0, (Object[]) parts);
		} catch (UnreadableException e) {
			Logger.log(LogLevel.ERROR, e);
			myAgent.doDelete();
		}
	}
}
