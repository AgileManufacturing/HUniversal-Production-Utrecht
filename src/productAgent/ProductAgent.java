package productAgent;

import equipletAgent.EquipletAgent;
import jade.core.AID;
import jade.core.Agent;
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
	ProduceBehaviour prodBehav = new ProduceBehaviour();
	
	private static final long serialVersionUID = 1L;

	// Private fields
	private Product _product;

	// CID variables
	private static int _cidCnt = 0;
	private String _cidBase;

	public int prodStep = 0;
	PlannerBehaviour planBehav = new PlannerBehaviour();
	EquipletAgent eqAgent = new EquipletAgent();
	protected void setup() {
		try {
			_product = (Product) getArguments()[0];

			addBehaviour(new OverviewBehaviour());

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
		int curProdStep = prodStep;
		planBehav.action(); // add the posibility to reschedule at the given prodStep
	}
	
	public void rescheduleAndRemoveEquiplet(){
		//int curProdStep = prodBehav.getCurrentProductionStep(); // TODO get the current production step from 'produceBehaviour'
		AID removeEQ = getAID(); // get the AID at which the rescheduling was needed
		planBehav.removeEquiplet(removeEQ);
		reschedule();
		// restart the planner behaviour at the curProdStep set by the produceBehaviour	
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
