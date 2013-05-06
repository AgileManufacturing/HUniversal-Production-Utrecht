/**
 * 
 */

package productAgent;

import jade.core.behaviours.CyclicBehaviour;

import java.util.List;

import newDataClasses.LogMessage;
import newDataClasses.Product;
import newDataClasses.ProductionStep;
import newDataClasses.ProductionStepStatus;

/**
 * 
 * @brief This behaviour is running when the production is about to start till
 *        the end of the production.
 * @author Theodoor de Graaff <theodoor.degraaff@student.hu.nl>
 * 
 */
public class ProduceBehaviour extends CyclicBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Product _product;

	/**
	 */
	public ProduceBehaviour(){
	}

	/*
	 * (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#action()
	 */
	@Override
	public void action(){
		// TODO Auto-generated method stub
	}

	static void canProductionStepStart(ProductionStep step){
		step.setStatus(ProductionStepStatus.STATE_PRODUCING);
	}

	void productionStepEnded(ProductionStep step, boolean succes,
			List<LogMessage> log){
		_product.add(log);
		if (succes){
			step.setStatus(ProductionStepStatus.STATE_DONE);
		} else{
			step.setStatus(ProductionStepStatus.STATE_FAILED);
		}
		// TODO If latest step, check all steps done, then end production?
	}
}
