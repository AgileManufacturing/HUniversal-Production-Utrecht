package simulation.mas.equiplet;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import simulation.util.Capability;
import simulation.util.Position;
import simulation.util.Triple;
import agents.equiplet_agent.EquipletAgent;

public class Equiplet extends EquipletAgent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
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
	private boolean jobBecameReady;

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
		this.jobBecameReady = false;
	}

	/**
	 * TODO fix this so that the name can be called from jade.Agent so agent can communicate with each other. note that this need to fix setup or constructor of equipletagent to
	 * set the name
	 * 
	 * @return
	 */
	@Deprecated
	public String getEquipletName() {
		return name;
	}

	public Position getPosition() {
		return position;
	}

	public EquipletState getEquipletState() {
		return state;
	}

	public List<String> getServices() {
		ArrayList<String> services = new ArrayList<>();
		for (Capability capability : capabilities) {
			services.add(capability.getService());
		}
		return services;
	}

	public boolean isCapable(String service, Map<String, Object> criteria) {
		for (Capability capability : capabilities) {
			if (capability.getService().equalsIgnoreCase(service)) {
				// Map<String, Object> limitations =
				// capability.getLimitations();
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

	public List<Job> getHistory() {
		return history;
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
			// System.out.println("load " + time + "= 1 - (" + busy + "/" +
			// window + ") =" + (1.0 - (busy * 1.0 / window)) + ", schedule " +
			// schedule);

			// the busy time can't be large than the window, altough there could
			// be an overlap of max a job duration
			return 1.0 - 1.0 * (Math.min(busy, window) / window);
		} else if (schedule.size() > 0) {
			Job job = schedule.first();

			if (job.getDeadline() >= time && job.getStartTime() <= time + window) {
				// System.out.println("load in window " + time + "= 1 - (" +
				// job.getDuration() + "/" + window + ") =" + (1.0 -
				// (job.getDuration() * 1.0 / window)) + ", schedule " +
				// schedule);
				return 1.0 - (1.0 * job.getDuration() / window);
			} else {
				// System.out.println("load out window " + time +
				// "= 1, schedule " + schedule);
				return 1.0;
			}
		} else {
			return 1.0;
		}
	}

	/**
	 * the first possible time there is enough room in the schedule to perform a service
	 * 
	 * @param time
	 *            the first possible time from which to look
	 * @param service
	 *            the name of the service to be performed
	 * @return the first available time
	 */
	public double available(double time, String service) {
		/*
		 * double productionTime = estimateJob(service); for (Job job : schedule) {
		 * 
		 * }
		 */
		// TODO fix this so job can be scheduled in between jobs instead of
		// alway behind the last
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
			return String.format("%s:[state=%s, capabilities=%s, time of breakdown=%.2f, executing=%s, schedule=%d, history=%d]", name, state, capabilities, timeBreakdown,
					executing, schedule.size(), history.size());
		} else {
			return String.format("%s:[state=%s, capabilities=%s, executing=%s, schedule=%d, history=%d]", name, state, capabilities, (state == EquipletState.IDLE ? "null"
					: executing), schedule.size(), history.size());
		}
	}

	/**
	 * The simulation need to check if the just scheduled product step is going to be executed
	 * 
	 * @param product
	 *            name
	 * @param service
	 * @param criteria
	 * @return is the job
	 */
	public boolean isExecutingStep(String product, String service, Map<String, Object> criteria) {
		return (executing != null && executing.getProductAgentName().equalsIgnoreCase(product) && executing.getService().equalsIgnoreCase(service) && executing.getCriteria().equals(
				criteria));
	}

	public boolean isExecuting() {
		return executing != null;
	}

	public String getExecutingProduct() {
		return executing.getProductAgentName();
	}

	public void executeJob(double time) {
		state = EquipletState.BUSY;
		executing = schedule.pollFirst();
		executing.updateStartTime(time);
		System.out.printf("EQ:%s starts at %.2f with executing job: %s\n", name, time, executing);

		execute(executing);
	}

	protected void execute(Job toExecuteJob) {

	}

	protected void executionFinished() {

	}

	/**
	 * Notify a product is arrived by the equiplet and is ready to be let the equiplet execute his product step
	 * 
	 * @param time
	 *            of the the product arrival
	 * @param product
	 *            name of the product agent
	 * @param service
	 *            name of the service that the equiplet is ask to perform for the product
	 */
	public void notifyProductArrived(double time, String product, String service) {
		// check if it is needed to also check the criteria
		// TODO possible service not necessary, if making the constraint that a
		// product can only have one job ready by an equiplet
		for (Job job : schedule) {
			if (job.getProductAgentName().equalsIgnoreCase(product) && job.getService().equalsIgnoreCase(service) && !job.isReady()) {
				job.setReady();
				break;
			}
		}

		// TODO combine the set ready loop above with the possibility to execute
		// a job that is later in the schedule but can already be performed

		// execute the first job in the schedule if the job is ready
		if (state == EquipletState.IDLE && schedule.first().isReady()) {
			historyUpdate(time);
			executeJob(time);
		} else if (!isExecuting() && schedule.first().isReady() && state == EquipletState.ERROR) {
			// executing = schedule.pollFirst(); //FOUT
			jobBecameReady = true;
			System.out.printf("EQ:%s product %s going to be executed after repair\n", name, product);
		} else {
			System.out.printf("EQ:%s product %s is added to waiting products\n", name, product);
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
			System.out.printf("EQ:%s job %s should have finished at %.2f, notifyJobFinished() : [breakdown=%.0f, finished=%.0f, broken=%.0f, executing=%s]\n", name, executing,
					time, timeBreakdown, timeShouldHaveFinished, timeBroken, executing);
		} else if (hasFinishedDuringRepair()) {
			System.out.printf("EQ:%s job should finished but delayed by breakdown, should still %.2f be executed : [breakdown=%.0f, finished=%.0f, broken=%.0f, executing=%s]\n",
					name, getRemainingTime(), timeBreakdown, timeShouldHaveFinished, timeBroken, executing);
			state = EquipletState.BUSY;
		} else {
			executing.updateDueTime(time);
			history.add(executing);

			System.out.printf("EQ:%s finished with job %s\n", name, executing);

			historyUpdate(time);
			if (!schedule.isEmpty() && schedule.first().isReady()) {
				executeJob(time);
			} else {
				state = EquipletState.IDLE;
				executing = null;
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
		System.out.printf("EQ:%s is broken down at %.2f\n", name, time);
		historyUpdate(time);
		state = EquipletState.ERROR;
		timeBroken = -1;
		timeBreakdown = time;
		timeShouldHaveFinished = -1;
		System.out.printf("EQ:%s error debug notifyBreakdown() [breakdown=%.0f, finished=%.0f, broken=%.0f, executing=%s]\n", name, timeBreakdown, timeShouldHaveFinished,
				timeBroken, executing);
	}

	/**
	 * The notify that the equiplet is repaired if the equiplet has finished during the repair, remember the time the equiplet has broken
	 * 
	 * @param time
	 *            of repair
	 */
	public void notifyRepaired(double time) {
		historyUpdate(time);

		if (jobBecameReady) {
			jobBecameReady = false;
			executeJob(time);

			timeBroken = -1;
			timeBreakdown = -1;
			timeShouldHaveFinished = -1;
		} else if (isExecuting()) {
			System.out.printf("EQ:%s is repaired at %.2f and continue with job %s \n", name, time, executing);
			timeBroken = time - timeBreakdown;
			state = EquipletState.BUSY;
		} else {
			System.out.printf("EQ:%s is repaired at %.2f \n", name, time);
			state = EquipletState.IDLE;

			timeBroken = -1;
			timeBreakdown = -1;
			timeShouldHaveFinished = -1;
		}
		System.out.printf("EQ:%s error debug notifyRepaired() [breakdown=%.0f, finished=%.0f, broken=%.0f, executing=%s]\n", name, timeBreakdown, timeShouldHaveFinished,
				timeBroken, executing);
	}

	/**
	 * The simulation need the know the time the equiplet was broken to set the finished event of the executing job over the remaining time of the job
	 * 
	 * @return time the equiplet was broken
	 */
	public double getTimeBroken() {
		return timeBroken;
	}

	/**
	 * The simulation has set the finished event over the remaining time of the executing job
	 * 
	 * @return the time the equiplet should have finished
	 */
	@Deprecated
	public double getShouldHaveFinish() {
		return timeShouldHaveFinished;
	}

	/**
	 * The equiplet should have finished with the job while he was being repaided The function gives the simulation the remaining time of the job which still needs to be executed
	 * 
	 * @return the remaining process time of the job
	 */
	public double getRemainingTime() {
		return timeShouldHaveFinished - timeBreakdown;
	}

	/**
	 * Ask if the Equiplet has finished the job during the time the equiplet was broken. If this is the case, the time should have finished is set
	 * 
	 * @return if the equipet finished the job
	 */
	public boolean hasFinishedDuringRepair() {
		return timeShouldHaveFinished > 0; // FOUT
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

	public boolean isNewJobReady() {
		return jobBecameReady;
	}

	public boolean wasBroken() {
		if (timeBroken > 0) {
			double x = timeBroken;
			timeBroken = -1;
			timeBreakdown = -1;
			timeShouldHaveFinished = -1;
			return true;
		} else {
			return false;
		}
	}
}
