package MAS.simulation.mas.product;

import java.util.LinkedList;

import org.json.JSONException;

import MAS.product.ProductAgent;
import MAS.product.ProductState;
import MAS.product.ProductStep;
import MAS.product.ProductionStep;
import MAS.simulation.simulation.ISimulation;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;

public class ProductAgentSim extends ProductAgent implements IProductSim {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ISimulation simulation;

	// TODO created has become a Tick, so maybe it is not needed to store here, but let the real agent handle the tick, or describe way this is here
	private Tick created;

	public ProductAgentSim(ISimulation simulation, LinkedList<ProductStep> productSteps, Position position, Tick time, Tick deadline) {
		try {
			Object[] args = new Object[] { Parser.parseProductConfiguration(productSteps, position, deadline) };
			setArguments(args);
		} catch (JSONException e) {
			System.err.printf("PA: failed to create product: %s.\n", e.getMessage());
		}

		this.simulation = simulation;
		this.created = time;
	}

	@Override
	public void kill() {
		// TODO handle properly, let the equiplet know that I been killed
		System.out.printf("PA:%s terminating\n", getLocalName());
		doDelete();
	}

	@Override
	public void onProductArrived(Tick time) {
		super.onProductArrived(time);
	}

	@Override
	public Position getPosition() {
		return super.getPosition();
	}

	@Override
	public Tick getCreated() {
		return created;
	}

	@Override
	public Tick getDeadline() {
		return super.getDeadline();
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
		ProductionStep step = getCurrentStep();
		simulation.notifyProductProcessing(getLocalName(), step.getEquipletName(), step.getService(), step.getIndex());
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
