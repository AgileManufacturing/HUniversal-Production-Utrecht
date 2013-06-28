package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import rexos.mas.behaviours.ReceiveBehaviour;

public class SubInformerBehaviour extends ReceiveBehaviour {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private AID _targetEquiplet;
	
	private BehaviourCallback _behaviourCallback;

	public SubInformerBehaviour(Agent myAgent, BehaviourCallback behavioirCallback, AID targetEquiplet) {
		super(myAgent);
		_behaviourCallback = behavioirCallback;
		_targetEquiplet = targetEquiplet;
	}

	@Override
	public void handle(ACLMessage message) {
		// TODO Auto-generated method stub
		
	}
	
	

}
