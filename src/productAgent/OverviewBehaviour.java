package productAgent;

import jade.core.behaviours.Behaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.ThreadedBehaviourFactory;

@SuppressWarnings("serial")
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
	}

	// Change behaviour
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
	}
}
