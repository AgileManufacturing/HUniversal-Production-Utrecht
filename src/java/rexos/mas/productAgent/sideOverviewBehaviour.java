package rexos.mas.productAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class sideOverviewBehaviour extends CyclicBehaviour {
	ACLMessage msg;
	ProductAgent pa = new ProductAgent();
	
	@SuppressWarnings("unused")
	private class receiveMsgBehaviour extends CyclicBehaviour {
		
		private receiveMsgBehaviour(){
		}
		
		@Override
		public void action() {
			ACLMessage msg = myAgent.receive();
			if (msg != null) {
				WaitMsgBehaviour behaviour = new WaitMsgBehaviour(msg);
			} else {
				block();
			}
		}
	}
	
	private class WaitMsgBehaviour extends OneShotBehaviour {
		ACLMessage msg;

		public WaitMsgBehaviour(ACLMessage msg) {
			this.msg = msg;
		}
	
		public void action(){
			try {
				switch(msg.getOntology()){
					case "waiting":
						@SuppressWarnings("unused")
						OneShotBehaviour waitingBehaviour = new OneShotBehaviour(){
							public void action(){
								
							}
						};
					case "reschedule":
						@SuppressWarnings("unused")
						OneShotBehaviour reschedule = new OneShotBehaviour(){
							public void action(){
								pa.reschedule();
							}
						};
						break;
					case "reschedule++":
						@SuppressWarnings("unused")
						OneShotBehaviour rescheduleAndRemoveEquiplet = new OneShotBehaviour(){
							public void action(){
								pa.rescheduleAndRemoveEquiplet();
							}
						};
						break;	
					default:
						return;
					}
				}catch(Exception e){
					System.out.println("" + e);
				}
			}
		}
	

	@Override
	public void action() {
		// TODO Auto-generated method stub
		
	}
}