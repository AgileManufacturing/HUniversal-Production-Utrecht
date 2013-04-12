package productAgent;

import newDataClasses.ProductionStep;
import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class sideOverviewBehaviour extends CyclicBehaviour {
	ACLMessage msg;
	ProductAgent pa = new ProductAgent();
	
	private class receiveMsgBehaviour extends CyclicBehaviour {
		ParallelBehaviour par;
		
		private receiveMsgBehaviour(){
			par = new ParallelBehaviour(
					ParallelBehaviour.WHEN_ALL);
		}
		
		@Override
		public void action() {
			ACLMessage msg = myAgent.receive();
			if (msg != null) {
				WaitMsgBehaviour behaviour = new WaitMsgBehaviour(msg);
				par.addSubBehaviour(behaviour);
			} else {
				block();
			}
			myAgent.addBehaviour(par);
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
						OneShotBehaviour waitingBehaviour = new OneShotBehaviour(){
							public void action(){
								
							}
						};
					case "reschedule":
						OneShotBehaviour reschedule = new OneShotBehaviour(){
							public void action(){
								pa.reschedule();
							}
						};
						break;
					case "reschedule++":
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