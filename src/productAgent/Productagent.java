/*
 * @Author Alexander
 * @Version 0.1
 * 
 * Initial product agent. 
 * Added functions to generate a Conv. Id based on the agents localname.
 */

package productAgent;

import jade.core.Agent;
import newDataClasses.Product;

public class ProductAgent extends Agent {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	//Private fields
	private Product _product;
	
	//CID variables
	private static int _cidCnt = 0;
	private String _cidBase;

	protected void setup() {
		try {
			_product = (Product) getArguments()[0];

			InformerBehaviour nb = new InformerBehaviour();
			PlannerBehaviour pb = new PlannerBehaviour();
			addBehaviour(pb);

			System.out.println("I spawned as a product agent");

		} catch (Exception e) {
			System.out.println("Exited with: " + e);
			doDelete();
		}
	}

	/*
	 * Generates an unique conversation id based on the agents localname,
	 * the objects hashcode and the current time.
	 */
	public String generateCID() {
		if (_cidBase == null) {
			_cidBase = getLocalName() + hashCode() + System.currentTimeMillis()
					% 10000 + "_";
		}
		return _cidBase + (_cidCnt++);
	}

	public Product getProduct() {
		return this._product;
	}
}
