/**
 * 
 */
package rexos.mas.productAgent;

import java.util.List;

import jade.core.behaviours.OneShotBehaviour;
import rexos.mas.newDataClasses.LogMessage;
import rexos.mas.newDataClasses.Product;
import rexos.mas.newDataClasses.ProductionStep;

/**
 * 
 * @brief This behaviour is running when the production is about to start till the end of the production.
 * @author Theodoor de Graaff <theodoor.degraaff@student.hu.nl>
 * 
 */
public class ProduceBehaviour extends OneShotBehaviour{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Product _product; 
	private int currentProductionStep;
	
	/**
	 * @return the _currentproductionstep
	 */
	public ProductionStep get_currentproductionstep() {
		return _product.getProduction().getProductionSteps().get(currentProductionStep);
	}

	/**
	 */
	public ProduceBehaviour() {
		
	}
	
	/* (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#action()
	 */
	@Override
	public void action() {
		// TODO Auto-generated method stub
	}

	void productionStepEnded(ProductionStep step, boolean succes, List<LogMessage> log)
	{	
		currentProductionStep = step.getId();
		//_product.add(log);
	}
}
