package productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import newDataClasses.Product;
import newDataClasses.ProductionStep;

/**
 * @Author Alexander
 * @Version 0.1
 * 
 *          Initial product agent. Added functions to generate a Conv. Id based
 *          on the agents localname.
 */
public class ProductAgent extends Agent {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	// Private fields
	private Product _product;
	private InformerBehaviour ib;

	// CID variables
	private static int _cidCnt = 0;
	private String _cidBase;

	@SuppressWarnings("serial")
	protected void setup() {
		try {
			_product = (Product) getArguments()[0];

			ib = new InformerBehaviour();
			PlannerBehaviour pb = new PlannerBehaviour();
			ProduceBehaviour pbz = new ProduceBehaviour();
			SocketBehaviour sb = new SocketBehaviour();

			addBehaviour(pb);

			addBehaviour(new CyclicBehaviour() {
				@Override
				public void action() {
					if (ib.done()) {
						outPutProductStepList();
					} else {
						System.out.println(" -- Informer Not done -- ");
					}
				}
			});
			System.out.println("I spawned as a product agent");

		} catch (Exception e) {
			System.out.println("Exited with: " + e);
			doDelete();
		}
	}

	/*
	 * Generates an unique conversation id based on the agents localname, the
	 * objects hashcode and the current time.
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
	
	public void setProduct(Product value) {
		this._product = value;
	}

	public void outPutProductStepList() {
		for (ProductionStep stp : this.getProduct().getProduction()
				.getProductionSteps()) {
			for (AID aid : this.getProduct().getProduction()
					.getProductionEquipletMapping()
					.getEquipletsForProductionStep(stp.getId())) {
				System.out
						.println("Step: " + stp.getId() + " has equiplets:\n");
				System.out.println(aid.getLocalName());
			}
		}
	}
}
