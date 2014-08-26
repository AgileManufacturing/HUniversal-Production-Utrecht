package simulation.mas.product;

import java.util.LinkedList;

import org.json.JSONException;

import simulation.simulation.ISimulation;
import simulation.util.Parser;
import simulation.util.Position;

public class ProductAgentSim extends ProductAgent implements IProductSim {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ISimulation simulation;
	private double created;

	public ProductAgentSim(ISimulation simulation, LinkedList<ProductStep> productSteps, Position position, double time) {
		try {
			Object[] args = new Object[] { Parser.parseProductConfiguration(productSteps, position) };
			setArguments(args);
		} catch (JSONException e) {
			System.err.printf("EA: failed to create equiplet: %s.\n", e.getMessage());
		}
		
		this.simulation = simulation;
		this.created = time;
	}

	@Override
	public void onProductArrived(double time) {
		super.onProductArrived(time);
	}

	@Override
	public Position getPosition() {
		return super.getPosition();
	}

	@Override
	public double getCreated() {
		return created;
	}

	@Override
	protected void onProductStepFinished() {
		super.onProductStepFinished();

		// After regular behaviour when a product step is finished, inform also the simulation
		if (getProductState() == ProductState.FINISHED) {
			// notify the simulation that the product is finished
			simulation.notifyProductFinished(getLocalName());
		} else if (getProductState() == ProductState.TRAVELING) {
			// notify the simulation that the product is traveling
			simulation.notifyProductTraveling(getLocalName(), getCurrentStep().getEquipletName());
		}
	}

	@Override
	protected void onProductProcessing() {
		super.onProductProcessing();

		// notify the simulation that processing begins
		simulation.notifyProductProcessing(getLocalName(), getCurrentStep().getEquipletName(), getCurrentStep().getService());
	}

	@Override
	protected void schedulingFinished(boolean succeeded) {
		System.out.println("scheduling finished");
		// let the simulation know that the creation of product agent failed
		if (succeeded) {
			simulation.notifyProductCreated(getLocalName(), getCurrentStep().getEquipletName());
		} else {
			simulation.notifyProductCreationFailed(getLocalName());
		}

	}
}
