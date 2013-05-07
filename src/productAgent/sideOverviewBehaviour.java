/**
 * @file SideOverviewBehaviour.java
 * @brief Behaviour used by the product agent to converse. Needed for
 *        rescheduling, removing an equiplet and when the product travels
 *        between equiplets.
 * @date Created: 08-04-2013
 * 
 * @author Arno Derks
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

package productAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class SideOverviewBehaviour extends CyclicBehaviour{
	ACLMessage msg;
	ProductAgent pa;

	@SuppressWarnings("unused")
	private class receiveMsgBehaviour extends CyclicBehaviour{
		private receiveMsgBehaviour(){
		}

		@Override
		public void action(){
			ACLMessage msg = myAgent.receive();
			if (msg != null){
				WaitMsgBehaviour behaviour = new WaitMsgBehaviour(msg);
			} else{
				block();
			}
		}
	}

	private class WaitMsgBehaviour extends OneShotBehaviour{
		ACLMessage msg;

		public WaitMsgBehaviour(ACLMessage msg){
			this.msg = msg;
		}

		@Override
		public void action(){
			try{
				switch(msg.getOntology()){
				// the traveling time between equiplets
				case "journey":
					@SuppressWarnings("unused")
					OneShotBehaviour waitingBehaviour = new OneShotBehaviour(){
						@Override
						public void action(){
						}
					};
					break;
				// reschedule the product
				case "reschedule":
					@SuppressWarnings("unused")
					OneShotBehaviour reschedule = new OneShotBehaviour(){
						@Override
						public void action(){
							pa.reschedule();
						}
					};
					break;
				// remove the equiplet with a malfunction and reschedule the
				// product
				case "equiplet malfunction":
					@SuppressWarnings("unused")
					OneShotBehaviour rescheduleAndRemoveEquiplet = new OneShotBehaviour(){
						@Override
						public void action(){
							pa.rescheduleAndRemoveEquiplet();
						}
					};
					break;
				default:
					return;
				}
			} catch(Exception e){
				System.out.println("" + e);
			}
		}
	}

	@Override
	public void action(){
		// TODO Auto-generated method stub
	}
}