package simulation.mas;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import simulation.util.Capability;
import simulation.util.Position;
import simulation.util.Triple;

public class Equiplet {

	private static final double SAFETY_FACTOR = 1;
	private String name;
	private Position position;
	private List<Capability> capabilities;

	private TreeSet<Job> schedule;
	private List<Job> history;

	private Map<String, Double> productionTimes;

	// Equiplet state
	private EquipletState state;
	private Job executing;

	// Simulation variables
	private double timeShouldHaveFinished;
	private double timeBreakdown;
	private double timeBroken;

	private Triple<Double, Double, Double> statistics;
	private double lastHistoryUpdate;

	public Equiplet(String name, Position position, List<Capability> capabilities, Map<String, Double> productionTimes) {
		this.name = name;
		this.position = position;
		this.capabilities = capabilities;
		this.schedule = new TreeSet<>();
		this.history = new ArrayList<>();
		this.productionTimes = productionTimes;

		this.state = EquipletState.IDLE;
		this.executing = null;

		lastHistoryUpdate = 0;
		statistics = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);

		this.timeShouldHaveFinished = -1;
		this.timeBreakdown = -1;
		this.timeBroken = -1;
	}

	public String getName() {
		return name;
	}

	public Position getPosition() {
		return position;
	}

	public EquipletState getState() {
		return state;
	}

	public List<String> getServices() {
		ArrayList<String> services = new ArrayList<>();
		for (Capability capability : capabilities) {
			services.add(capability.getService());
		}
		return services;
	}

	public boolean isCapable(String service, HashMap<String, Object> criteria) {
		for (Capability capability : capabilities) {
			if (capability.getService().equalsIgnoreCase(service)) {
				// Map<String, Object> limitations = capability.getLimitations();
				// TODO check limitations
				return true;
			}
		}
		return false;
	}

	public boolean providesService(String service) {
		return productionTimes.containsKey(service);
	}

	public double estimateService(String service) {
		return productionTimes.get(service) * SAFETY_FACTOR;
	}

	public int getWaiting() {
		int waiting = 0;
		for (Job job : schedule) {
			if (job.isReady()) {
				waiting++;
			}
		}
		return waiting;
	}

	public int getScheduled() {
		return schedule.size();
	}

	public int executedJobs() {
		return history.size();
	}

	public boolean schedule(double start, double deadline, String product, String service, Map<String, Object> criteria) {
		double duration = estimateService(service);

		if (schedule.subSet(new Job(start, 0), true, new Job(start, duration), true).isEmpty()) {
			Job job = new Job(service, product, criteria, start, start + duration, deadline);
			schedule.add(job);
			return true;
		} else {
			return false;
		}
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
	public double load(double time, double window) {
		if (schedule.size() > 1) {
			double busy = 0.0;

			for (Job job : schedule) {
				if (job.getDeadline() >= time && job.getDeadline() <= time + window) {
					busy += job.getDuration();
				}
			}
			// System.out.println("load " + time + "= 1 - (" + busy + "/" + window + ") =" + (1.0 - (busy * 1.0 / window)) + ", schedule " + schedule);

			// the busy time can't be large than the window, altough there could be an overlap of max a job duration 
			return 1.0 - 1.0 * (Math.min(busy, window) / window);
		} else if (schedule.size() > 0) {
			Job job = schedule.first();

			if (job.getDeadline() >= time && job.getStartTime() <= time + window) {
				// System.out.println("load in window " + time + "= 1 - (" + job.getDuration() + "/" + window + ") =" + (1.0 - (job.getDuration() * 1.0 / window)) + ", schedule " + schedule);
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
	 * the first possible time there is enough room in the schedule to perform a
	 * service
	 * 
	 * @param time
	 *            the first possible time from which to look
	 * @param service
	 *            the name of the service to be performed
	 * @return the first available time
	 */
	public double available(double time, String service) {
		/*
		 * double productionTime = estimateJob(service);
		 * for (Job job : schedule) {
		 * 
		 * }
		 */
		// TODO fix this so job can be scheduled in between jobs instead of alway behind the last
		if (schedule.size() > 0) {
			Job job = schedule.last();
			return job.getDeadline();
		} else {
			return time;
		}
	}

	@Override
	public String toString() {
		if (state == EquipletState.ERROR) {
			return String.format("%s:[state=%s, capabilities=%s, time broken=%.2f, schedule=%s, history=%s]", name, state, capabilities, timeBroken, schedule, history);
		} else {
			return String.format("%s:[state=%s, capabilities=%s, executing=%s, schedule=%s, history=%s]", name, state, capabilities, (state == EquipletState.IDLE ? "null" : executing), schedule, history);
		}
	}

	/**
	 * The simulation need to check if the just scheduled product step is going
	 * to be executed
	 * 
	 * @param product
	 *            name
	 * @param service
	 * @param criteria
	 * @return is the job
	 */
	public boolean isExecutingStep(String product, String service, HashMap<String, Object> criteria) {
		return (executing != null && executing.getProductAgent().equalsIgnoreCase(product) && executing.getService().equalsIgnoreCase(service) && executing.getCriteria().equals(criteria));
	}

	public boolean isExecuting() {
		return executing != null;
	}

	public String getExecutingProduct() {
		return executing.getProductAgent();
	}

	/**
	 * Notify a product is arrived by the equiplet and is ready to be let the
	 * equiplet execute his product step
	 * 
	 * @param time
	 *            of the the product arrival
	 * @param product
	 *            name of the product agent
	 * @param service
	 *            name of the service that the equiplet is ask to perform for
	 *            the product
	 */
	public void notifyProductArrived(double time, String product, String service) {
		// check if it is needed to also check the criteria
		// TODO possible service not necessary, if making the constraint that a product can only have one job ready by an equiplet
		for (Job job : schedule) {
			if (job.getProductAgent().equalsIgnoreCase(product) && job.getService().equalsIgnoreCase(service) && !job.isReady()) {
				job.setReady();
				break;
			}
		}

		// TODO combine the set ready loop above with the possibility to execute a job that is later in the schedule but can already be performed

		// execute the first job in the schedule if the job is ready
		if (state == EquipletState.IDLE &&  schedule.first().isReady()) {
			historyUpdate(time);
			state = EquipletState.BUSY;
			executing = schedule.pollFirst();
		} else if (state == EquipletState.ERROR) {
			executing = schedule.pollFirst();
		}
			
	}

	/**
	 * Notify the executing job is finished
	 * 
	 * @param time
	 *            the job finished
	 */
	public void notifyJobFinished(double time) {
		if (state == EquipletState.ERROR) {
			timeShouldHaveFinished = time;
		} else {
			timeBroken = -1;
			timeBreakdown = -1;
			timeShouldHaveFinished = -1;

			historyUpdate(time);
			if (isExecuting()) {
				history.add(executing);

				if (!schedule.isEmpty() && schedule.first().isReady()) {
					state = EquipletState.BUSY;
					executing = schedule.pollFirst();
				} else {
					state = EquipletState.IDLE;
					executing = null;
				}
			} else {
				executing = null;
				state = EquipletState.IDLE;
				System.out.println("FAIL: job finished a non executing job");
			}
		}
	}

	/**
	 * Set the time of the breakdown of the equiplet
	 * 
	 * @param time
	 *            of the breakdown
	 */
	public void notifyBreakdown(double time) {
		historyUpdate(time);
		state = EquipletState.ERROR;
		this.timeBreakdown = time;
	}

	/**
	 * The notify that the equiplet is repaired
	 * if the equiplet has finished during the repair, remember the time the
	 * equiplet has broken
	 * 
	 * @param time
	 *            of repair
	 */
	public void notifyRepaired(double time) {
		historyUpdate(time);
		if (!hasFinishedDuringRepair()) {
			state = EquipletState.BUSY;
			this.timeBroken = time - timeBreakdown;
			this.timeBreakdown = -1;
			this.timeShouldHaveFinished = -1;
		} else if (isExecuting()) {
			state = EquipletState.BUSY;
		} else {
			state = EquipletState.IDLE;
		}
	}

	/**
	 * The simulation need the know the time the equiplet was broken to set the
	 * finished event of the executing job over the remaining time of the job
	 * 
	 * @return time the equiplet was broken
	 */
	public double getTimeBroken() {
		return timeBroken;
	}

	/**
	 * The simulation has set the finished event over the remaining time of the
	 * executing job
	 * 
	 * @return the time the equiplet should have finished
	 */
	public double getShouldHaveFinish() {
		return timeShouldHaveFinished;
	}

	/**
	 * Ask if the Equiplet has finished the job during the time the equiplet was
	 * broken. If this is the case, the time should have finished is set
	 * 
	 * @return if the equipet finished the job
	 */
	public boolean hasFinishedDuringRepair() {
		return timeShouldHaveFinished > 0;
	}

	private void historyUpdate(double time) {
		double elapsed = time - lastHistoryUpdate;
		lastHistoryUpdate = time;
		if (state == EquipletState.IDLE) {
			statistics.first += elapsed;
		} else if (state == EquipletState.BUSY) {
			statistics.second += elapsed;
		} else if (state == EquipletState.ERROR) {
			statistics.third += elapsed;
		}
	}

	public Triple<Double, Double, Double> getStatistics(double time) {
		historyUpdate(time);
		return statistics;
	}

	public int getExecuted() {
		return history.size();
	}

	public boolean validateSchedule(double time) {
		if (executing == null) {
			for (Job job : schedule) {
				if (job.isReady() && job.getStartTime() < time) {
					return false;
				}
			}
		}
		return true;
	}
}
