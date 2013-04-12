package ProductAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.ThreadedBehaviourFactory;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class sideOverviewBehaviour extends CyclicBehaviour {
	ACLMessage msg;
	
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
						
					case "reschedule":
						
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