package productAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.ThreadedBehaviourFactory;
import jade.lang.acl.ACLMessage;

@SuppressWarnings("serial")
public class OverviewBehaviour extends CyclicBehaviour {
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
						myAgent.addBehaviour(new OneShotBehaviour(myAgent) {
						@Override
						public void action() {
							
						}
					});
				case "informerBehaviour":
					
				case "schedulerBehaviour":
					
				case "produceBehaviour":
					
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
			/*changeBehaviour(1);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
=======
public class OverviewBehaviour extends Behaviour {
	private ThreadedBehaviourFactory _tbf;
	private ProductAgent _productAgent;
	private int _currentState;

	/* Behaviours */
	private PlannerBehaviour _plannerBehaviour;
	private SchedulerBehaviour _schedulerBehaviour;

	public OverviewBehaviour() {
		_tbf = new ThreadedBehaviourFactory();
		_productAgent = (ProductAgent) myAgent;
	}

	@Override
	public void action() {
		SequentialBehaviour s = new SequentialBehaviour();
		
		s.addSubBehaviour(new PlannerBehaviour());
		s.addSubBehaviour(new InformerBehaviour());
		s.addSubBehaviour(new SchedulerBehaviour());
		s.addSubBehaviour(new ProduceBehaviour());
		
		myAgent.addBehaviour(s);
>>>>>>> 0f9656d7da80656e44a728d88910aaa02934bd69
	}
	// Change behaviour
<<<<<<< HEAD
	public void changeBehaviour(int state) throws InterruptedException{
		switch(state){
			case 0: // planner
				final OneShotBehaviour osbPlan = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(tbf.wrap(osbPlan));					
					}
				};				
				break;
			case 1: // informer
				final OneShotBehaviour osbInfo = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(tbf.wrap(osbInfo));
					}
				};				
				break;
			case 2: // scheduler
				final OneShotBehaviour osbSched = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(tbf.wrap(osbSched));
					}
				};
				break;
			case 3: // production
				final OneShotBehaviour osbProd = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(tbf.wrap(osbProd));			
					}
				};
				break;
			case 4: // waiting
				myAgent.waitUntilStarted();
				break;
			default:
				return;
		}
	}*/
	
	// Reschedule
	public void reschedule(){
		// reschedule from the given step, step represents the point at which the production has to be resumed
=======
	private void changeBehaviour(int state) {
		switch (state) {
		case 0: // Planner
			break;
		case 1: // Informer
			break;
		case 2: // Scheduler
			break;
		case 3: // Producing
			break;
		default: // Listening for other msgs
			return;
		}
	}

	/* (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#done()
	 */
	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return false;
>>>>>>> 0f9656d7da80656e44a728d88910aaa02934bd69
	}
			@Override
			public void action() {
				// TODO Auto-generated method stub
				
			}
}
