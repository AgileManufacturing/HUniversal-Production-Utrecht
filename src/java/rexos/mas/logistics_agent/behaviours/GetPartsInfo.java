/**
 * @file GetPartsInfoResponse.java
 * @brief 
 * @date Created: 22 apr. 2013
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
package rexos.mas.logistics_agent.behaviours;

import java.io.IOException;
import java.util.HashMap;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import rexos.mas.behaviours.ReceiveOnceBehaviour;
import rexos.mas.data.Position;

/**
 * @author Peter
 * 
 */
public class GetPartsInfo extends ReceiveOnceBehaviour {
	private static final long serialVersionUID = 1L;

	/**
	 * @param a
	 */
	public GetPartsInfo(Agent a, String conversationId) {
		this(a, 2000, conversationId);
	}

	/**
	 * @param a
	 */
	public GetPartsInfo(Agent a, int millis, String conversationId) {
		super(a, millis, MessageTemplate.and(
				MessageTemplate.MatchOntology("GetPartsInfo"),
				MessageTemplate.MatchConversationId(conversationId)));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	@Override
	public void handle(ACLMessage message) {
		if (message != null) {
			try {
				System.out.format("%s GetPartsInfo%n", myAgent.getLocalName());
				// TODO probleem: als PA parameters aangeeft is dat soms
				// part-gebonden: bal A op pos
				// x en bal B op pos y. Daarom moeten de parts of als parameters
				// worden aangegeven, samen met de pos, of de part-gebonden
				// parameters komen bij de
				// parts array in: niet nice.
				// TODO probleem: een part kan twee of meer part-gebonden
				// parameters
				// hebben (van en naar pos). Een hashmap kan dus niet zomaar
				// tenzij Position wordt uitgebreid met een van en naar pos.

				Integer[] partIds = (Integer[]) message.getContentObject();
				HashMap<Integer, Position> partsParameters = new HashMap<Integer, Position>();

				int x = 0;
				for (int partId : partIds) {
					partsParameters.put(partId, new Position(x++, 1, 3));
				}

				ACLMessage reply = message.createReply();
				reply.setOntology("GetPartsInfoResponse");
				reply.setContentObject(partsParameters);
				myAgent.send(reply);
			} catch (UnreadableException | IOException e) {
				e.printStackTrace();
				myAgent.doDelete();
			}
		} else {
			myAgent.removeBehaviour(this);
		}
	}
}
