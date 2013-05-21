
package java.rexos.mas.productAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class SideOverviewBehaviour extends CyclicBehaviour{
	ACLMessage msg;
	ProductAgent _productAgent;

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
				// reschedule the product
				case "reschedule":
					@SuppressWarnings("unused")
					OneShotBehaviour reschedule = new OneShotBehaviour(){
						@Override
						public void action(){
							_productAgent.reschedule();
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
							_productAgent.rescheduleAndRemoveEquiplet();
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