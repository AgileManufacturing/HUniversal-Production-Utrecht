package productAgent;

import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ThreadedBehaviourFactory;

@SuppressWarnings("serial")
public class OverviewBehaviour extends CyclicBehaviour {
	int state = 0;
	PlannerBehaviour planBehav = new PlannerBehaviour();
	InformerBehaviour infoBehav = new InformerBehaviour();
	SchedulerBehaviour schedBehav = new SchedulerBehaviour();
	ProduceBehaviour prodBehav = new ProduceBehaviour();
	private ThreadedBehaviourFactory tbf = new ThreadedBehaviourFactory();
	
	public void action(){
		try {
			changeBehaviour(1);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	// Change behaviour
	public void changeBehaviour(int state) throws InterruptedException{
		switch(state){
			case 0: // planner
				final OneShotBehaviour osbPlan = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(osbPlan);					
						tbf.wrap(osbPlan);
					}
				};				
				break;
			case 1: // informer
				final OneShotBehaviour osbInfo = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(osbInfo);
						tbf.wrap(osbInfo);
					}
				};				
				break;
			case 2: // scheduler
				final OneShotBehaviour osbSched = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(osbSched);
						tbf.wrap(osbSched);
					}
				};
				break;
			case 3: // production
				final OneShotBehaviour osbProd = new OneShotBehaviour(){
					public void action(){
						myAgent.addBehaviour(osbProd);
						tbf.wrap(osbProd);
					}
				};
				break;
			case 4: // waiting
				myAgent.waitUntilStarted();
				break;
			default:
				return;
		}
	}
	
	// Reschedule
	public void reschedule(){
		// reschedule from the given step, step represents the point at which the production has to be resumed
	}
}
