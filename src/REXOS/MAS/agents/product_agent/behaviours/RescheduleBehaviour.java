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
package agents.product_agent.behaviours;

import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;

import agents.data_classes.BehaviourStatus;
import agents.data_classes.Product;
import agents.data_classes.Production;
import agents.data_classes.ProductionStep;
import agents.data_classes.StepStatusCode;
import agents.product_agent.BehaviourCallback;
import agents.product_agent.ProductAgent;

import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

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
		Logger.log(LogLevel.INFORMATION, "RESCHEDULING!");
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
