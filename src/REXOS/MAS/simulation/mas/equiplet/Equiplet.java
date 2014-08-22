package simulation.mas.equiplet;

import jade.core.Agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import simulation.mas.product.ProductStep;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Triple;

public abstract class Equiplet extends Agent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	// private static final double SAFETY_FACTOR = 1;

	protected Position position;
	protected List<Capability> capabilities;

	// Equiplet state
	protected TreeSet<Job> schedule;
	protected EquipletState state;
	protected Job executing;
	protected List<Job> history;
	private Map<String, Double> productionTimes;

	/**
	 * initialize the equiplet
	 * 
	 * @param position
	 *            of the equiplet in the grid
	 * @param capabilities
	 *            of the equiplet
	 * @param productionTimes
	 *            estimate of duration of the providing services
	 */
	protected void init(Position position, List<Capability> capabilities) {
		this.position = position;
		this.capabilities = capabilities;

		this.state = EquipletState.IDLE;
		this.executing = null;
		this.schedule = new TreeSet<>();
		this.history = new ArrayList<>();

		this.productionTimes = new HashMap<>();
		for (Capability capability : capabilities) {
			productionTimes.put(capability.getService(), capability.getDuration());
		}
	}

	/**
	 * 
	 * @return the position of the equiplet
	 */
	protected Position getPosition() {
		return position;
	}

	/**
	 * 
	 * @return the state of the equiplet
	 */
	protected EquipletState getEquipletState() {
		return state;
	}

	/**
	 * @return whether the equiplet is executing a job
	 */
	protected boolean isExecuting() {
		return executing != null;
	}

	protected synchronized Pair<String, Double> getExecuting() {
		return new Pair<String, Double>(executing.getProductAgentName(), executing.getStartTime());
	}

	/**
	 * Get the number of job that are scheduled for the equiplet
	 * 
	 * @return number of jobs
	 */
	protected synchronized int getScheduled() {
		return schedule.size();
	}

	/**
	 * Get the number of jobs waiting to be executed i.e. ready for execution A
	 * job is ready for executing when the product arrived by the equiplet
	 * 
	 * @return number of jobs ready to executed
	 */
	protected synchronized int getWaiting() {
		int waiting = 0;
		for (Job job : schedule) {
			if (job.isReady()) {
				waiting++;
			}
		}
		return waiting;
	}

	/**
	 * Get the number of jobs executed by the equiplet
	 * 
	 * @return number of jobs
	 */
	protected synchronized int getExecuted() {
		return history.size();
	}

	/**
	 * give an estimate of the duration of the service
	 * 
	 * @param service
	 *            name
	 * @return the duration
	 */
	protected double estimateService(String service) {
		return productionTimes.get(service);
	}

	/**
	 * Checks whether a job is ready for execution TODO check not only the first
	 * in the schedule but also after if job can be executed earlier than
	 * planned, which increases
	 * complexity
	 * 
	 * @return if there is job ready for executing
	 */
	protected boolean jobReady() {
		return !schedule.isEmpty() && schedule.first().isReady();
	}

	/**
	 * check whether the equiplet can execute a list of product steps within a time frame
	 * this returns the list of services that can be performed @see {@code isCapable}, with the estimate duration of the service and a list of possible time frames
	 * the time frame is a list of times from which the equiplet is free until the equiplet busy again where the deadline is the time till when is looked
	 * 
	 * @param time
	 *            of the first possibility to perform the product steps
	 * @param deadline
	 *            of the product steps
	 * @param productSteps
	 *            the list of product steps to be performed
	 * @return
	 */
	public List<Triple<Integer, Double, List<Pair<Double, Double>>>> canExecute(double time, double deadline, List<ProductStep> productSteps) {
		// answer :: List of services < index in production path, estimate production time, List of from and until time when possible>
		List<Triple<Integer, Double, List<Pair<Double, Double>>>> answer = new ArrayList<>();

		for (ProductStep productStep : productSteps) {
			if (isCapable(productStep.getService(), productStep.getCriteria())) {
				int index = productStep.getIndex();
				double duration = estimateService(productStep.getService());
				List<Pair<Double, Double>> available = available(time, duration, deadline);
				answer.add(new Triple<Integer, Double, List<Pair<Double, Double>>>(index, duration, available));
			}
		}
	
		return answer;
	}

	/**
	 * check whether the equiplet is capable to perform a service with certain criteria
	 * 
	 * @param service
	 * @param criteria
	 * @return whether the equiplet is capable to perform the service
	 */
	protected boolean isCapable(String service, Map<String, Object> criteria) {
		for (Capability capability : capabilities) {
			if (capability.getService().equalsIgnoreCase(service)) {
				// Map<String, Object> limitations = capability.getLimitations();
				// TODO check limitations
				return true;
			}
		}
		return false;
	}

	/**
	 * the first possible time there is enough room in the schedule to perform a service
	 * 
	 * @param time
	 *            the first possible time from which to look
	 * @param duration
	 *            an estimate of time the equiplet is checked for availability
	 * @return a list of time it is possible to plan the duration TODO can only
	 *         plan in end of schedule
	 */
	protected List<Pair<Double, Double>> available(double time, double duration, double deadline) {
		List<Pair<Double, Double>> available = new ArrayList<Pair<Double, Double>>();
		double start = time;
		if (isExecuting()) {
			start = Math.max(start, executing.getDue());
		}

		
		if (schedule.isEmpty()) {
			available.add(new Pair<Double, Double>(start, deadline));
		} else {
			
		}
			
			Iterator<Job> it = schedule.iterator();
			while (it.hasNext()) {
				Job job = it.next();

				if (job.getStartTime() > start) {

					if (job.getStartTime() - start > duration) {

						if (job.getStartTime() < deadline) {
							available.add(new Pair<Double, Double>(start, job.getStartTime()));
							start = job.getDue();
						} else {
							available.add(new Pair<Double, Double>(start, deadline));
							break;
						}
					} else {
						start = job.getDue();
					}
				}
				
				if (!it.hasNext()) {
					available.add(new Pair<Double, Double>(start, deadline));
				}
			}
		
		return available;
		
		/*
		 * 
		if (schedule.size() > 0) {
			Job job = schedule.last();
			available.add(new Pair<Double, Double>(job.getDue(), deadline));
		} else if (isExecuting()) {
			available.add(new Pair<Double, Double>(executing.getDue(), deadline));
		} else {
			available.add(new Pair<Double, Double>(time, deadline));
		}
		return available;
		 */
	}

	/**
	 * calculate the load of the equiplet from a certain time with a window
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	protected double load(double time, double window) {
		// TODO validate correctness
		if (schedule.size() > 1) {
			double busy = 0.0;

			for (Job job : schedule) {
				if (job.getDeadline() >= time && job.getDeadline() <= time + window) {
					busy += job.getDuration();
				}
			}
			// System.out.println("load " + time + "= 1 - (" + busy + "/" + window + ") =" + (1.0 - (busy * 1.0 / window)) + ", schedule " + schedule);

			// the busy time can't be large than the window, although there could
			// be an overlap of max a job duration
			return 1.0 - 1.0 * (Math.min(busy, window) / window);
		} else if (schedule.size() > 0) {
			Job job = schedule.first();

			if (job.getDeadline() >= time && job.getStartTime() <= time + window) {
				// System.out.println("load in window " + time + "= 1 - (" + job.getDuration() + "/" + window + ") =" + (1.0 - (job.getDuration()*1.0/window))
				// + ", schedule " + schedule);
				return 1.0 - (1.0 * job.getDuration() / window);
			} else {
				// System.out.println("load out window " + time + "= 1, schedule " + schedule);
				return 1.0;
			}
		} else {
			return 1.0;
		}
	}

	/**
	 * The actual start of executing a job
	 * 
	 * @param job
	 */
	protected void execute(Job job) {
		
	}
}
