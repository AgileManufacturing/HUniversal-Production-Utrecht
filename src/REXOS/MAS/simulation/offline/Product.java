package MAS.simulation.offline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;

import MAS.product.ProductState;
import MAS.product.ProductStep;
import MAS.product.ProductionStep;
import MAS.product.SchedulingException;
import MAS.simulation.mas.product.IProductSim;
import MAS.simulation.util.Settings;
import MAS.util.Position;
import MAS.util.Tick;

class Product implements IProductSim {
	private Sim simulation;
	private String name;

	private Tick created;
	protected LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	protected ArrayList<ProductionStep> history;

	protected Position position;
	private Tick deadline;
	protected ProductState state;
	protected boolean reschedule;

	public Product(Sim simulation, String name, Tick time, LinkedList<ProductStep> steps, Position startPosition, Tick deadline) {
		this.simulation = simulation;
		this.name = name;
		this.created = time;

		this.position = startPosition;
		this.productSteps = steps;
		this.productionPath = new LinkedList<>();

		this.deadline = deadline;
		this.state = ProductState.SCHEDULING;

		this.history = new ArrayList<>();
		this.reschedule = false;

	}

	public void schedule(Tick time) {
		try {
			simulation.scheduleProduct(name, time, position, productSteps, deadline);
		} catch (SchedulingException e) {
			System.out.printf("PA:%s failed to schedule: %s\n", name, e.getMessage());
			simulation.notifyProductCreationFailed(name);
		}
	}

	@Override
	public Tick getCreated() {
		return created;
	}

	@Override
	public Tick getDeadline() {
		return deadline;
	}

	@Override
	public Position getPosition() {
		return position;
	}

	private ProductionStep getCurrentStep() {
		return productionPath.peek();
	}

	@Override
	public void kill() {
		// let the gc do its job
	}

	@Override
	public String toString() {
		return String.format("%s:[state=%s created=%s, product steps=%s, current step=%s]", name, state, created, Arrays.toString(productSteps.toArray()), productionPath.peek());
	}

	// @Override
	public void onProductArrived(Tick time) {
		// change state from travelling to ready
		state = ProductState.WAITING;

		ProductionStep step = getCurrentStep();
		position = step.getPosition();
		simulation.informProductArrived(time, name, step.getEquipletName());

		if (state == ProductState.WAITING) {
			simulation.notifyProductShouldStart(name, step.getStart(), step.getIndex());
		}
	}

	protected void onProductStepFinished(Tick time) {
		// remove the first production step as this is finished
		ProductionStep step = productionPath.pop();
		step.setFinished(time);
		history.add(step);

		if (productionPath.isEmpty()) {
			state = ProductState.FINISHED;// notify the simulation that the product is finished
			simulation.notifyProductFinished(name);
		} else {
			state = ProductState.TRAVELING;
			performNextStep();
		}
	}

	protected void schedulingFinished(Tick time, boolean succeeded, LinkedList<ProductionStep> steps) {
		System.out.printf("PA:%s scheduling finished %b. \n", name, succeeded);
		state = ProductState.TRAVELING;
		productionPath = steps;

		// let the simulation know that the creation of product agent failed
		if (reschedule && succeeded) {
			reschedule = false;
			simulation.notifyProductRescheduled(name, getCurrentStep().getEquipletName(), succeeded);
		} else if (reschedule) {
			// Tick deadline = getDeadline().add(getDeadline().minus(getCreated()).multiply(++retry));
			// System.out.printf("PA:%s try rescheduling again at %s with new deadline %s. \n", name, time, deadline);
			// reschedule(time, deadline);
			System.out.println("product" + this);
			throw new IllegalArgumentException("PA:" + name + " failed to reschedule deadline=" + getDeadline().add(getDeadline().minus(getCreated())));
		} else if (succeeded) {
			simulation.notifyProductCreated(name, getCurrentStep().getEquipletName());
		} else {
			simulation.notifyProductCreationFailed(name);
		}
	}

	protected void performNextStep() {
		// notify the simulation that the product is traveling
		simulation.notifyProductTraveling(name, getCurrentStep().getEquipletName());

	}

	protected void onProductProcessing(Tick time) {
		state = ProductState.PROCESSING;

		// notify the simulation that processing begins
		ProductionStep step = getCurrentStep();
		step.updateStart(time);
		simulation.notifyProductProcessing(name, step.getEquipletName(), step.getService(), step.getIndex());
	}

	@Override
	public void onProductStarted(Tick time, int index) {
		if (getCurrentStep().getIndex() == index && (state == ProductState.ERROR || state == ProductState.SCHEDULING || state == ProductState.TRAVELING)) {
			// throw new IllegalArgumentException("on product started event given in wrong state " + getProductState() + "");
		} else {
			Tick newDeadline = deadline.add(deadline.minus(getCreated()));

			System.out.printf("PA:%s on product started event [time=%s, index=%d, state=%s, deadline=%s, new deadline=%s, current step=%s, productionPath].\n", name, time, index, state, deadline, newDeadline, getCurrentStep(), productionPath);
			// Check if the equiplet is in the correct state, if Processing everything is correct.
			// When product is waiting check whether the index of current product steps matches
			// The product should never be in other states than waiting and processing
			if (state == ProductState.WAITING && getCurrentStep().getIndex() == index) {
				System.out.printf("PA:%s rescheduling remaining production steps %s.\n", name, productionPath);

				// release all the time slots planned by equiplets
				release(time);

				// reschedule
				reschedule(time, newDeadline);
			}
		}

		if (!reschedule) {
			simulation.notifyProductRescheduled(false);
		}
	}

	/**
	 * release all the product steps (time slots) planned by the equiplets
	 * 
	 * @param time
	 *            current time
	 */
	private void release(Tick time) {
		// release all the time slots planned by equiplets
		HashSet<String> equiplets = new HashSet<>();
		for (ProductionStep step : productionPath) {
			equiplets.add(step.getEquipletName());
		}

		simulation.informProductRelease(name, equiplets);
	}

	/**
	 * reschedule the remaining product steps
	 * 
	 * @param time
	 *            current time
	 * @param deadline
	 *            of the product (may be increased whenever first attempt isn't
	 *            successful)
	 */
	protected void reschedule(Tick time, Tick deadline) {
		this.state = ProductState.SCHEDULING;
		this.reschedule = true;
		LinkedList<ProductStep> steps = new LinkedList<>();
		for (ProductionStep step : productionPath) {
			steps.add(step.getProductStep());
		}

		if (productionPath.size() != steps.size()) {
			throw new IllegalArgumentException("WTF? for work no more?");
		}

		if (productionPath.size() > 0 && productionPath.size() + history.size() != Settings.MEAN_PRODUCT_STEPS) {
			System.err.println(this);
			throw new IllegalArgumentException("WTF? were dit it go??");
		}

		try {
			simulation.scheduleProduct(name, time, position, steps, deadline);
		} catch (SchedulingException e) {
			// infinite recursion?
			reschedule(time, deadline.add(1000000));
		}
	}

	public void informProductProcessing(Tick time, String equiplet) {
		state = ProductState.PROCESSING;

		// notify the simulation that processing begins
		ProductionStep step = getCurrentStep();
		step.updateStart(time);
		if (!step.getEquipletName().equals(equiplet)) {
			throw new IllegalArgumentException("Equiplet name: "+ equiplet + " differs from "+ step.getEquipletName() + " in step "+ step + " and path: " + productionPath);
		}
		simulation.notifyProductProcessing(name, step.getEquipletName(), step.getService(), step.getIndex());
	}

	public void informProductStepFinished(Tick time, int index) {
		// remove the first production step as this is finished
		ProductionStep step = productionPath.pop();
		step.setFinished(time);
		history.add(step);

		if (productionPath.isEmpty()) {
			state = ProductState.FINISHED;
		} else {
			state = ProductState.TRAVELING;
			performNextStep();
		}

		// After regular behaviour when a product step is finished, inform also the simulation
		if (state == ProductState.FINISHED) {
			// notify the simulation that the product is finished
			simulation.notifyProductFinished(name);
			simulation.log(Settings.PRODUCT_LOG, name, "PA:" + name + " finished: " + history);
		}
	}
}
