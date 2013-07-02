
package rexos.mas.productAgent;

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
		/**
		 * Start wait message behavior
		 */
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

		/**
		 * Construct wait message behavior
		 * @param msg
		 */
		public WaitMsgBehaviour(ACLMessage msg){
			this.msg = msg;
		}
		
		/**
		 * Sets status of behavior
		 */
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
							//pa.rescheduleAndRemoveEquiplet();
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

	/**
	 * Parent performs action
	 */
	@Override
	public void action(){
		// TODO Auto-generated method stub
	}
}