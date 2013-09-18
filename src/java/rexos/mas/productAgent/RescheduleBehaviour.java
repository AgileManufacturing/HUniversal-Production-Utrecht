/**
 *
 * Project: PA-V2
 *
 * Package: rexos.mas.productAgent
 *
 * File: RescheduleBehaviour.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.productAgent;

import java.util.ArrayList;

import rexos.libraries.log.Logger;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import rexos.mas.data.StepStatusCode;
import rexos.mas.data.BehaviourStatus;

/**
 * @author Mike
 *
 */
public class RescheduleBehaviour extends Behaviour {
	private ProductAgent _productAgent;
	private BehaviourCallback _bc;

	public RescheduleBehaviour(Agent a, BehaviourCallback bc) {
		super(a);
		_productAgent = (ProductAgent) a;
		this._bc = bc;
	}

	/* (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#action()
	 */
	@Override
	public void action() {
		//System.out.println("RESCHEDULING!");
		Product product = _productAgent.getProduct();
		Production prod = product.getProduction();
		ArrayList<ProductionStep> steps = prod.getProductionSteps();
		
		for(ProductionStep step : steps) {
			if(step.getStatus() != StepStatusCode.DONE && step.getStatus() != StepStatusCode.PLANNED) {
				step.setStatus(StepStatusCode.RESCHEDULE);
				ACLMessage message = new ACLMessage(ACLMessage.INFORM);
				message.addReceiver(step.getUsedEquiplet());
				message.setOntology("AbortStep");
				message.setConversationId(step.getConversationId());
				myAgent.send(message);
			}
		}	
		
		prod.setProductionSteps(steps);
		product.setProduction(prod);
		_productAgent.setProduct(product);
		_bc.handleCallback(BehaviourStatus.COMPLETED, null);
	}
	

	/* (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#done()
	 */
	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return true;
	}
	
	
}
