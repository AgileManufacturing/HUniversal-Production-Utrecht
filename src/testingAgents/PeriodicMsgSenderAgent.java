/**
 * @file PeriodicMsgSenderAgent.java
 * @brief 
 * @date Created: 17 Apr 2013
 * 
 * @author Alexander
 * 
 * @section LICENSE
 * License: newBSD 
 *  
 * Copyright © 2012, HU University of Applied Sciences Utrecht. 
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
package testingAgents;

import java.util.Random;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.WakerBehaviour;
import jade.lang.acl.ACLMessage;

/**
 * @author Alexander
 * 
 *         Sends periodic msgs to test the product agent's parallel behaviour
 *         Sends an 'Reschedule' msg to force the product agent to reschedule.
 *         Sends an 'EquipletMalfunction' msg to fake the malfunctioning of an
 *         equiplet
 * 
 *         DISCLAIMER I DID NOT FOLLOW ANY CODING REGULATIONS. THIS IS NOT
 *         OFFICIAL PROJECT CODE, NOR WILL IT BE USED FOR ANYTHING OTHER THEN
 *         TESTING. COMMENTS ARE SCARCE AND CRAPPY
 * 
 *         USE AT OWN RISK.
 */
public class PeriodicMsgSenderAgent extends Agent {

	private static final long serialVersionUID = 1L;
	private boolean debug = true;

	// CID variables
	private static int _cidCnt = 0;
	private String _cidBase;

	@SuppressWarnings("serial")
	protected void setup() {
		try {

			addBehaviour(new WakerBehaviour(this, getRandomInt(30000)) {
				protected void onWake() {
					
					ACLMessage message = new ACLMessage(ACLMessage.INFORM);
					message.addReceiver(new AID("pa1", AID.ISLOCALNAME));
					message.setConversationId(generateCID());
					
					if(getRandomBoolean()){
						if (debug)
							System.out.println("Sending a reschedule msg.");
						message.setOntology("Reschedule");
					} else {
						if (debug)
							System.out.println("Sending a Move msg.");
						message.setOntology("MoveToEQ");
					}
					
					myAgent.send(message);
				}
			});

		} catch (Exception e) {
			System.out.println("PeriodicMsgSenderAgent Exited with: " + e);
			doDelete();
		}
	}

	public boolean getRandomBoolean() {
		Random random = new Random();
		return random.nextBoolean();
	}

	public int getRandomInt(int r) {
		Random random = new Random();
		return random.nextInt(r);
	}

	/*
	 * Generates an unique conversation id based on the agents localname, the
	 * objects hashcode and the current time.
	 */
	public String generateCID() {
		if (_cidBase == null) {
			_cidBase = getLocalName() + hashCode() + System.currentTimeMillis()
					% 10000 + "_";
		}
		return _cidBase + (_cidCnt++);
	}

}
