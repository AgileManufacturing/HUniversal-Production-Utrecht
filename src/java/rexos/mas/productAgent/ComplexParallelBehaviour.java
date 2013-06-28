package rexos.mas.productAgent;

import jade.core.Agent;
import jade.core.behaviours.ParallelBehaviour;

public class ComplexParallelBehaviour extends ParallelBehaviour {

	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private BehaviourCallback _behaviourCallback;

	public ComplexParallelBehaviour(Agent myAgent, int endCondition, BehaviourCallback behaviourCallback) {
		super(myAgent, endCondition);
		_behaviourCallback = behaviourCallback;
	}
	
	
	@Override
	protected boolean checkTermination(boolean currentDone,
            int currentResult) {
		return false;
		
	}
	
	
	
	
	
	
	
	
	
}
