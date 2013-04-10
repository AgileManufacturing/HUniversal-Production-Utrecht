package productAgent;

import jade.core.behaviours.SequentialBehaviour;

@SuppressWarnings("serial")
public class OverviewBehaviour extends SequentialBehaviour {
	int state = 0;
	PlannerBehaviour planBehav = new PlannerBehaviour();
	InformerBehaviour infoBehav = new InformerBehaviour();
	SchedulerBehaviour schedBehav = new SchedulerBehaviour();
	ProduceBehaviour prodBehav = new ProduceBehaviour();
	// The product agent is created
	OverviewBehaviour(ProductAgent pa){		
		
	}
	
	
	
	// Change behaviour
	public void changeBehaviour(int state){
		switch(state){
			case 0: // start
				//
				break;
			case 1: // planner
				startPlanBehav();
				break;
			case 2: // informer
				startInfoBehav();
				break;
			case 3: // scheduler
				startSchedBehav();
				break;
			case 4: // production
				startProdBehav();
				break;
			case 5: // waiting
				//
				break;
			default:
				return;
		}
	}
		
	// Go to the planner behaviour
	public void startPlanBehav(){
		planBehav.action();
		OverviewBehaviour.this.block();
	}
	
	// Go to the informer behaviour
	public void startInfoBehav(){
		infoBehav.action();
		OverviewBehaviour.this.block();
	}
	
	// Go to the scheduler behaviour
	public void startSchedBehav(){
		schedBehav.action();
		OverviewBehaviour.this.block();
	}
	
	// Go to the produce behaviour
	public void startProdBehav(){
		prodBehav.action();
		OverviewBehaviour.this.block();
	}
	
	
	// Reschedule
	public void reschedule(){
		// reschedule from the given step, step represents the point at which the production has to be resumed
	}
}
