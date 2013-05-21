/**
 * @file GatewayAgent.java
 * @brief
 * @date Created: 02-04-2013
 * 
 * @author Ricky van Rijn
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package rexos.mas.gatewayAgent;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.util.Logger;
import jade.wrapper.AgentController;

public class GateWayAgent extends Agent{
	/**
	 * SerialVersion
	 */
	private static final long serialVersionUID = 1;
	private Logger myLogger = Logger.getMyLogger(getClass().getName());

	private void creationOfAgents(){
		String name = "Alice";
		jade.wrapper.AgentContainer c = getContainerController();
		try{
			AgentController a = c.createNewAgent(name, "agent.com.Henk", null);
			a.start();
		} catch(Exception e){
			System.out.println(e.getMessage());
		}
	}

	@SuppressWarnings("serial")
	private class WaitPingAndReplyBehaviour extends CyclicBehaviour{
		public WaitPingAndReplyBehaviour(Agent a){
			super(a);
		}

		@Override
		public void action(){
			ACLMessage msg = myAgent.receive();
			if (msg != null){
				creationOfAgents();
			} else{
				block();
			}
		}
	} // END of inner class WaitPingAndReplyBehaviour

	@SuppressWarnings("unused")
	private class XMLReader{
		public void parse(String Message){
		}
	}

	@Override
	protected void setup(){
		// Registration with the DF
		DFAgentDescription dfd = new DFAgentDescription();
		ServiceDescription sd = new ServiceDescription();
		sd.setType("SmallTalkAgent");
		sd.setName(getName());
		sd.setOwnership("RickyVanRijn");
		dfd.setName(getAID());
		dfd.addServices(sd);
		try{
			DFService.register(this, dfd);
			WaitPingAndReplyBehaviour PingBehaviour = new WaitPingAndReplyBehaviour(
					this);
			addBehaviour(PingBehaviour);
		} catch(FIPAException e){
			myLogger.log(Logger.SEVERE, "Agent " + getLocalName()
					+ " - Cannot register with DF", e);
			doDelete();
		}
	}
}
