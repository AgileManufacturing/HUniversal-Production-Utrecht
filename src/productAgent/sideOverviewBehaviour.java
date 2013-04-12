package productAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.ThreadedBehaviourFactory;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class sideOverviewBehaviour extends CyclicBehaviour {
	int state = 0;
	PlannerBehaviour planBehav = new PlannerBehaviour();
	InformerBehaviour infoBehav = new InformerBehaviour();
	SchedulerBehaviour schedBehav = new SchedulerBehaviour();
	ProduceBehaviour prodBehav = new ProduceBehaviour();
	private ThreadedBehaviourFactory tbf = new ThreadedBehaviourFactory();
	ACLMessage msg;
	
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
					case "planningBehaviour":
						final OneShotBehaviour osbPlan = new OneShotBehaviour(){
							public void action(){
								myAgent.addBehaviour(tbf.wrap(osbPlan));					
							}
						};	
					case "informerBehaviour":
						final OneShotBehaviour osbInfo = new OneShotBehaviour(){
							public void action(){
								myAgent.addBehaviour(tbf.wrap(osbInfo));					
							}
						};	
					case "schedulerBehaviour":
						final OneShotBehaviour osbSched = new OneShotBehaviour(){
							public void action(){
								myAgent.addBehaviour(tbf.wrap(osbSched));					
							}
						};	
					case "produceBehaviour":
						final OneShotBehaviour osbProd = new OneShotBehaviour(){
							public void action(){
								myAgent.addBehaviour(tbf.wrap(osbProd));					
							}
						};	
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