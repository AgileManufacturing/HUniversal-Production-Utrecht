/**
 * 
 */
package ProductAgent;

import jade.core.behaviours.OneShotBehaviour;
import newDataClasses.LogMessage;
import newDataClasses.Product;

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

	void productionStepEnded(boolean succes, LogMessage[] log)
	{	
		_product.add(log);
	}
}
