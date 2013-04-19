package rexos.mas.productAgent;

//import testingAgents.EquipletAgent;
import jade.core.AID;
import jade.core.Agent;
import rexos.mas.newDataClasses.Product;
import rexos.mas.newDataClasses.ProductionStep;

/**
 * @Author Alexander
 * @Version 0.1
 * 
 *          Initial product agent. Added functions to generate a Conv. Id based
 *          on the agents localname.
 */
public class ProductAgent extends Agent {
	
	private static final long serialVersionUID = 1L;
	
	
	
	// Private fields
	private Product _product;
	private OverviewBehaviour _overviewBehaviour;
	
	
	// CID variables
	private static int _cidCnt = 0;
	private String _cidBase;

	public int prodStep = 0;
	

	@Override
	protected void setup() {
		try {
			_product = (Product) getArguments()[0];
			
			_overviewBehaviour = new OverviewBehaviour();
			addBehaviour(_overviewBehaviour);

			System.out.println("I spawned as a product agent");

		} catch (Exception e) {
			System.out.println("Productagent exited with: " + e.getMessage());
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
	
	public void reschedule(){
		_overviewBehaviour.reschedule();
	}
	
	public void rescheduleAndRemoveEquiplet(){
		//remove equiplet first
		_overviewBehaviour.reschedule();
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
					.getEquipletsForProductionStep(stp.getId()).keySet()) {
				System.out.println("Step: " + stp.getId() + " has equiplets:\n");
				System.out.println(aid.getLocalName());
			}
		}
	}
}
