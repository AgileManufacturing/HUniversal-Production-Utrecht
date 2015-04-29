package MAS.simulation.offline;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import org.json.JSONObject;

import MAS.equiplet.Capability;
import MAS.equiplet.EquipletState;
import MAS.equiplet.Job;
import MAS.simulation.mas.equiplet.IEquipletSim;
import MAS.util.MASConfiguration;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;
import MAS.util.Tuple;
import MAS.util.Util;

public class Equiplet implements IEquipletSim {
	private Sim simulation;

	// Equiplet knowledge
	private String name;
	protected Position position;
	protected List<Capability> capabilities;

	// Equiplet state
	protected TreeSet<Job> schedule;
	protected EquipletState state;
	protected boolean reconfiguring;
	protected Job executing;
	protected TreeSet<Job> history;
	protected Map<String, Tick> productionTimes;

	// Simulation variables
	/**
	 * time the equiplet breaks down
	 */
	private Tick timeBreakdown;
	/**
	 * the time remaining for the executing job after a breakdown
	 */
	private Tick timeRemaining;

	// Simulation performances
	/**
	 * statistics Statistics contains the time the equiplet is in one of the
	 * states <BUSY, IDLE, ERROR>
	 * The states ERROR_READY and ERROR_FINISHED are counted as ERROR and
	 * ERROR_REPAIRED as BUSY
	 */
	private Triple<Tick, Tick, Tick> statistics;

	/**
	 * lastHistoryUpdate The last time the statistics is update to
	 * calculate the elapsed time between state changes
	 */
	private Tick lastStatisticsUpdate;

	/**
	 * scheduleLatency A list of differences between the time and the time
	 * scheduled
	 */
	private Map<Tick, Tick> scheduleLatency;

	protected Equiplet(Sim simulation, String name, Position position, List<Capability> capabilities) {
		this.simulation = simulation;
		this.name = name;

		this.position = position;
		this.capabilities = capabilities;

		this.state = EquipletState.IDLE;
		this.reconfiguring = false;
		this.executing = null;
		this.schedule = new TreeSet<>();
		this.history = new TreeSet<>();

		this.productionTimes = new HashMap<>();
		for (Capability capability : capabilities) {
			productionTimes.put(capability.getService(), capability.getDuration());
		}

		lastStatisticsUpdate = new Tick(0);
		statistics = new Triple<Tick, Tick, Tick>(new Tick(0), new Tick(0), new Tick(0));
		scheduleLatency = new HashMap<Tick, Tick>();
	}

	public EquipletState getEquipletState() {
		return state;
	}

	public Position getPosition() {
		return position;
	}

	@Override
	public List<Capability> getCapabilities() {
		return capabilities;
	}

	public String getExecutingProduct() {
		return executing.getProductAgentName();
	}

	/**
	 * Schedule a job
	 * 
	 * @param product
	 * @param index
	 *            of the products production path
	 * @param start
	 *            time
	 * @param deadline
	 *            of job
	 * @param service
	 *            to be performed
	 * @param criteria
	 *            of the job
	 * @return if it succeeded to schedule the job
	 */
	protected synchronized boolean schedule(String product, int index, Tick start, Tick deadline, String service, JSONObject criteria) {
		// do not schedule a job when going to be reconfigured
		if (reconfiguring) {
			throw new IllegalArgumentException("not able to schedule job when reconfiguring");
			// return false;
		}

		Tick duration = estimateService(service);
		System.out.printf("EA:%s schedule [product=%s, start=%s, duration=%s, deadline=%s, service=%s, criteria=%s]\n", name, product, start, duration, deadline, service, criteria);

		if (schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))).isEmpty()) {
			@SuppressWarnings("deprecation")
			Job job = new Job(index, product, service, criteria, start, start.add(duration), deadline);
			return schedule.add(job);
		} else {
			// this shouldn't yet occur (not in the simulation), a equiplet
			// should never give a product the available time which cannot be
			// scheduled
			System.err.println("\n----------\nstart=" + start + ", due=" + start.add(duration) + "\nSCHEDULE:\n" + schedule + "\n\n");
			throw new IllegalArgumentException("overlap schedule ");
		}
	}

	/**
	 * Update the statistics This method should be called before each state
	 * change
	 * 
	 * @param time
	 */
	private void historyUpdate(Tick time) {
		Tick elapsed = time.minus(lastStatisticsUpdate);
		lastStatisticsUpdate = time;
		if (state == EquipletState.BUSY || state == EquipletState.ERROR_REPAIRED) {
			statistics.first = statistics.first.add(elapsed);
		} else if (state == EquipletState.IDLE) {
			statistics.second = statistics.second.add(elapsed);
		} else if (state == EquipletState.ERROR || state == EquipletState.ERROR_READY || state == EquipletState.ERROR_FINISHED) {
			statistics.third = statistics.third.add(elapsed);
		}
	}

	@Override
	public String toString() {
		return String.format("%s:[state=%s, \tcapabilities=%s, \texecuting=%s, \tscheduled=%d, \twaiting=%d, \thistory=%d]", name, state, capabilities, (executing == null ? "null" : executing), schedule.size(), getWaiting(), history.size());
	}

	public String toFullString() {
		return String.format("%s:[state=%s, \tcapabilities=%s, \texecuting=%s, \tscheduled=%d, \twaiting=%d, \thistory=%d] \n\thistory=%s \n\tschedule=%s", name, state, capabilities, (executing == null ? "null" : executing), schedule.size(), getWaiting(), history.size(), Util.formatSet(history), Util.formatSet(schedule));
	}

	public boolean release(String product) {
		Collection<Job> jobs = new HashSet<>();
		for (Job job : schedule) {
			if (job.getProductAgentName().equals(product)) {
				jobs.add(job);
			}
		}
		return schedule.removeAll(jobs);
	}

	/**
	 * Checks whether a job is ready for execution TODO check not only the first
	 * in the schedule but also after if job
	 * can be executed earlier than planned, which increases complexity
	 * 
	 * @return if there is job ready for executing
	 */
	protected synchronized Job jobReady() {
		int counter = 0;
		Iterator<Job> it = schedule.iterator();
		while (it.hasNext()) {
			Job job = (Job) it.next();
			if (job.isReady()) {
				return job;
			}
			counter++;
			if (counter > MASConfiguration.QUEUE_JUMP) {
				break;
			}
		}
		return null;
	}

	/**
	 * Start with executing the first job in the schedule Note: that the first
	 * job in the schedule need to be ready TODO
	 * fix that the job can be performed earlier that scheduled.
	 * 
	 * @param start
	 *            time of the job
	 * @param job
	 *            to be executed
	 */
	private void executeJob(Tick time, Job job) {
		Tick latency = time.minus(job.getStartTime());
		scheduleLatency.put(time, latency);

		state = EquipletState.BUSY;
		executing = job;
		schedule.remove(job);

		executing.updateStartTime(time);
		simulation.informProductProcessing(time, job.getProductAgentName(), name);
		// simulation.notifyProductProcessing(job.getProductAgentName(), name, job.getService(), job.getIndex());
	}

	/**
	 * a product is arrived
	 * 
	 * @param product
	 * @param time
	 */
	public void notifyProductArrived(String product, Tick time) {
		historyUpdate(time);
		Job arrived = null;
		for (Job job : schedule) {
			if (job.getProductAgentName().equals(product)) {
				job.setReady();
				arrived = job;
				break;
			}
		}

		// start with the job that arrived exactly on time
		Job ready = arrived.getStartTime().equals(time) && MASConfiguration.RESCHEDULE ? arrived : jobReady();

		// TODO combine the set ready loop above with the possibility to execute
		// a job that is later in the schedule but can already be performed

		// execute the first job in the schedule if the job is ready
		if (state == EquipletState.IDLE && ready != null) {
			// begin with executing job that arrived
			executeJob(time, ready);
		} else if (state == EquipletState.ERROR && !isExecuting() && ready != null && !MASConfiguration.RESCHEDULE) {
			// Equiplet is still broken, but as soon as this is repaired it will
			// execute the first job in the schedule
			System.out.printf("EA:%s product %s going to be executed after repair\n", name, product);
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", name, product);
		}
	}

	public void notifyJobFinished(Tick time) {
		if (state == EquipletState.ERROR) {
			// the equiplet should have finished the job, but was broken down in the meantime
			// the equiplet has still a remaining time to continue after the equiplet is repaired
			state = EquipletState.ERROR_FINISHED;
			timeRemaining = time.minus(timeBreakdown);
			executing.updateDueTime(executing.getDue().add(timeRemaining));
			System.out.printf("EA:%s job %s should finished but delayed by breakdown, should still %s be executed after being repaired.\n", name, executing, timeRemaining);
		} else if (state == EquipletState.ERROR_REPAIRED) {
			// the equiplet should have finished with the job, but was broken down in the meantime,
			// the equiplet continues with executing the job
			System.out.printf("EA:%s job %s should finished but delayed by breakdown, should still %s be executed.\n", name, executing, timeRemaining);
			state = EquipletState.BUSY;
		} else if (state == EquipletState.BUSY) {
			// executing of the job is really finished and will continue with the next job if possible
			executing.updateDueTime(time);
			history.add(executing);
			historyUpdate(time);

			System.out.printf("EA:%s finished with job: %s\n", name, executing);

			String finishedProduct = executing.getProductAgentName();
			int index = executing.getIndex();
			executing = null;

			Job ready = jobReady();
			if (ready != null) {
				// if (!schedule.isEmpty() && jobReady()) {
				executeJob(time, ready);
			} else if (reconfiguring && schedule.isEmpty()) {
				state = EquipletState.RECONFIG;
				simulation.notifyReconfigReady(name);
			} else {
				state = EquipletState.IDLE;
			}

			// note that the inform processing is done before inform finished
			// this is because the simulation can delete the product agent (if chosen to do so for performance improvement)
			// therefore there is no guarantee that informing the product is a blocking as the acknowledge is send before notifying the simulation
			informProductStepFinished(finishedProduct, time, index);
		} else {
			throw new IllegalArgumentException("EQUIPLET: notify job finished not given in correct state: " + state);
		}
	}

	public void notifyBreakdown(Tick time) {
		if (state == EquipletState.ERROR || state == EquipletState.ERROR_READY || state == EquipletState.ERROR_FINISHED || state == EquipletState.RECONFIG) {
			throw new IllegalArgumentException("EQUIPLET: notify breakdown not given in correct state: " + state);
		}

		if (state == EquipletState.ERROR_REPAIRED) {
			timeBreakdown = time.minus(timeRemaining);
			System.out.printf("EA:%s is broken down at %s, substracting the previous broken time %s\n", name, time, timeRemaining);
		} else {
			timeBreakdown = time;
			System.out.printf("EA:%s is broken down at %s\n", name, time);
		}
		historyUpdate(time);
		state = EquipletState.ERROR;
	}

	public void notifyRepaired(Tick time) {
		if (state == EquipletState.IDLE || state == EquipletState.BUSY || state == EquipletState.ERROR_REPAIRED || state == EquipletState.RECONFIG) {
			throw new IllegalArgumentException("EQUIPLET: notify breakdown not given in correct state: " + state);
		}
		historyUpdate(time);

		if (state == EquipletState.ERROR_FINISHED) {
			// the equiplet has already a finished event received, but is now repaired and can continue with the job
			state = EquipletState.BUSY;
			// executing.updateDueTime(executing.getDue() + (time - timeBreakdown));
			System.out.printf("EA:%s is repaired at %s and continue with job %s, with %s time remaining.\n", name, time, executing, timeRemaining);
		} else if (state == EquipletState.ERROR_READY) {
			// in the time the equiplet was broken there is a product arrived that can be executed
			Job ready = jobReady();

			if (ready != null) {
				executeJob(time, ready);
			} else {
				System.out.printf("EA:%s the job that was ready although when queue jumping is on, job can schedule in front such that there is no job ready when repaired.\n", name);
				state = EquipletState.IDLE;
			}
		} else if (isExecuting()) {
			// the equiplet is executing a job and is repaired, but waits until a job finished event is received
			state = EquipletState.ERROR_REPAIRED;
			timeRemaining = time.minus(timeBreakdown);
			executing.updateDueTime(executing.getDue().add(timeRemaining));
			System.out.printf("EA:%s is repaired at %s and continue with job %s. The equiplet was %s broken, which is still remaining.\n", name, time, executing, timeRemaining);
		} else {
			Job ready = jobReady();
			if (ready != null) {
				// when the equiplet was in the error state there became a job ready which arrived before the breakdown
				// not sure if this could happen
				executeJob(time, ready);
				System.out.printf("EA:%s is repaired at %s and detect that a job has became ready: %s \n", name, time, executing);
			} else {
				// the equiplet has nothing to do and goes into IDLE state
				System.out.printf("EA:%s is repaired at %s \n", name, time);
				state = EquipletState.IDLE;
			}
		}
	}

	public void notifyReconfigured(List<Capability> capabilities) {
		this.capabilities = capabilities;
		System.out.printf("EA:%s reconfigure finished he has new capabilties %s \n", name, capabilities);
		if (schedule.isEmpty()) {
			reconfiguring = false;
			for (Capability capability : capabilities) {
				productionTimes.put(capability.getService(), capability.getDuration());
			}
			state = EquipletState.IDLE;
		} else {
			throw new IllegalArgumentException("Equiplet has not an empty schedule while being reconfigured");
		}
	}

	/**
	 * Inform the product agent the job executed form him is finished
	 * 
	 * @param product
	 *            agents address
	 */
	protected void informProductStepFinished(String product, Tick time, int index) {
		simulation.informProductStepFinished(product, time, index);
	}

	/**
	 * check whether the equiplet is capable to perform a service with certain
	 * criteria
	 * 
	 * @param service
	 * @param criteria
	 * @return whether the equiplet is capable to perform the service
	 */
	protected synchronized boolean isCapable(String service, JSONObject criteria) {
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

	/**
	 * give an estimate of the duration of the service
	 * 
	 * @param service
	 *            name
	 * @return the duration
	 */
	protected Tick estimateService(String service) {
		return productionTimes.get(service);
	}

	/**
	 * the first possible time there is enough room in the schedule to perform a
	 * service load = 1 - Sr / Sw
	 * 
	 * @param time
	 *            the first possible time from which to look
	 * @param duration
	 *            an estimate of time the equiplet is checked for availability
	 * @return a list of time it is possible to plan the duration TODO can only
	 *         plan in end of schedule
	 */
	protected synchronized List<Pair<Tick, Tick>> available(Tick time, Tick duration, Tick deadline) {
		List<Pair<Tick, Tick>> available = new ArrayList<Pair<Tick, Tick>>();

		// when ever behind on schedule, the latency would otherwise keep increasing
		Tick window = schedule.isEmpty() ? deadline : deadline.minus(time.minus(schedule.first().getStartTime()).max(0));
		//		System.err.println("fix: "+ deadline + " - (" + time + " - "+ (schedule.isEmpty() ? "null" : schedule.first().getStartTime()) + ") = " + deadline +
		//		" - "+(schedule.isEmpty() ? "null" : (time.minus(schedule.first().getStartTime()))) + " = "+ window);

		// not availale when going to be reconfigured
		if (reconfiguring) {
			return available;
		}

		Tick start = time;
		if (isExecuting()) {
			// when executing add 10% of the duration to the time to prevent
			// reschedules in the same timeslot
			start = start.max(executing.getDue()).add(executing.getDuration().multiply(0.1));
		}

		if (state == EquipletState.ERROR || state == EquipletState.ERROR_FINISHED) {
			// when broken down add the largest time slot to now to prevent
			// product plan there product steps to close
			start = start.max(time.add(Collections.max(productionTimes.values())));
		}

		if (schedule.isEmpty()) {
			available.add(new Pair<Tick, Tick>(start, window));
		} else {
			Iterator<Job> it = schedule.iterator();
			while (it.hasNext()) {
				Job job = it.next();

				if (job.getStartTime().greaterOrEqualThan(start)) {

					if (job.getStartTime().minus(start).greaterThan(duration)) {

						if (job.getStartTime().lessThan(window)) {
							available.add(new Pair<Tick, Tick>(start, job.getStartTime()));
							start = job.getDue();
						} else if (start.lessThan(window)) {
							available.add(new Pair<Tick, Tick>(start, window));
							break;
						} else {
							break;
						}
					} else {
						start = job.getDue();
					}
				} else {
					start = job.getDue().max(start);
				}

				if (!it.hasNext() && start.lessThan(window)) {
					available.add(new Pair<Tick, Tick>(start, window));
				}
			}
		}

		return available;
	}

	/**
	 * calculate the load of the equiplet from a certain time with a window load
	 * = 1 - Sr / Sw
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	public double load(Tick time, Tick window) {
		Tick sum = new Tick(0);

		for (Job job : schedule) {
			if (job.getStartTime().greaterOrEqualThan(time) && job.getStartTime().lessOrEqualThan(time.add(window)) || job.getDue().greaterThan(time) && job.getDue().lessOrEqualThan(time.add(window))) {
				sum = sum.add(job.getDue().min(time.add(window)).minus(job.getStartTime().max(time)));
			}

			if (job.getStartTime().greaterThan(time.add(window))) {
				break;
			}
		}

		// System.out.println("EA:" + getLocalName() + " load= 1 - " + sum +
		// " / " + window + " = " + (1 - sum / window) + " in " + schedule);

		// double precision error, dirty fix
		sum = new Tick(Math.round(sum.doubleValue() * 100000000) / 100000000);

		return 1 - sum.div(window).doubleValue();
	}

	/**
	 * calculate the load of the history of the equiplet from a certain time
	 * with a window
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	public double loadHistory(Tick time, Tick window) {
		Tick sum = new Tick(0);

		if (isExecuting()) {
			sum = executing.getDue().min(time.add(window)).minus(executing.getStartTime().max(time));
		}

		Iterator<Job> iterator = history.descendingIterator();
		while (iterator.hasNext()) {
			Job job = iterator.next();
			if (job.getStartTime().greaterOrEqualThan(time) && job.getStartTime().lessOrEqualThan(time.add(window)) || job.getDue().greaterThan(time) && job.getDue().lessOrEqualThan(time.add(window))) {
				sum = sum.add(job.getDue().min(time.add(window)).minus(job.getStartTime().max(time)));
			} else if (job.getDue().lessThan(time)) {
				// the jobs are outside the scope of the load window
				break;
			}
		}

		if (!MASConfiguration.KEEP_FULL_EQUIPLET_HISTORY) {
			removeUnnecessaryHistory(time, window);
		}

		// double precision error, dirty fix, can use BigDecimal although
		// performance
		sum = new Tick(Math.round(sum.doubleValue() * 100000000) / 100000000);

		return 1 - sum.div(window).doubleValue();
	}

	/**
	 * remove unnecessary history to reduce memory usuage
	 * 
	 * @param time
	 * @param window
	 */
	private void removeUnnecessaryHistory(Tick time, Tick window) {
		Iterator<Job> iterator = history.iterator();
		while (iterator.hasNext()) {
			Job job = iterator.next();
			if (job.getDue().lessThan(time.minus(window))) {
				iterator.remove();
			} else {
				break;
			}
		}
	}

	/**
	 * the equiplet will be reconfigured when he is ready
	 * 
	 * @param capabilities
	 */
	@Override
	public void reconfigureStart() {
		System.out.printf("EA:%s reconfigure with capabilities %s \n", name, capabilities);
		reconfiguring = true;

		if (state == EquipletState.IDLE && schedule.isEmpty()) {
			state = EquipletState.RECONFIG;
			simulation.notifyReconfigReady(name);
		}
	}

	/**
	 * @return whether the equiplet is executing a job
	 */
	public boolean isExecuting() {
		return executing != null;
	}

	/**
	 * information of the executing job, i.e. the service with the duration
	 * 
	 * @return the executing job
	 */
	protected Pair<String, Tick> getExecuting() {
		return new Pair<String, Tick>(executing.getProductAgentName(), executing.getStartTime());
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
	 * job is ready for executing when the
	 * product arrived by the equiplet
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
	protected int getExecuted() {
		return history.size();
	}

	/**
	 * @return the difference between the start time and the planned start time
	 */
	@Override
	public Map<Tick, Tick> getLatency() {
		return scheduleLatency;
	}

	/**
	 * Retrieve the statistics of the equiplet
	 * 
	 * @param time
	 *            for last update to include time of question
	 * @return time the equiplet is <BUSY, IDLES, ERROR>
	 */
	@Override
	public Triple<Tick, Tick, Tick> getStatistics(Tick time) {
		historyUpdate(time);
		return statistics;
	}

	/**
	 * Get the complete schedule of the equiplet, the job which are executed,
	 * executing and to be executed
	 * The schedule contains a list of jobs with the start and end time and the
	 * product agent for whom the job is (to be) executed
	 * 
	 * @return a list of jobs
	 */
	@Override
	public List<Triple<String, Tick, Tick>> getCompleteSchedule() {
		List<Triple<String, Tick, Tick>> data = new ArrayList<Triple<String, Tick, Tick>>();
		for (Job job : history) {
			data.add(new Triple<String, Tick, Tick>(job.getProductAgentName(), job.getStartTime(), job.getDue()));
		}
		if (isExecuting()) {
			data.add(new Triple<String, Tick, Tick>(executing.getProductAgentName(), executing.getStartTime(), executing.getDue()));
		}
		for (Job job : schedule) {
			data.add(new Triple<String, Tick, Tick>(job.getProductAgentName(), job.getStartTime(), job.getDue()));
		}
		return data;
	}

	/**
	 * Get the schedule of the equiplet, the jobs to be executed
	 * 
	 * @return a list of jobs
	 */
	@Override
	public List<Triple<String, Tick, Tick>> getSchedule() {
		List<Triple<String, Tick, Tick>> data = new ArrayList<Triple<String, Tick, Tick>>();
		if (isExecuting()) {
			data.add(new Triple<String, Tick, Tick>(executing.getProductAgentName(), executing.getStartTime(), executing.getDue()));
		}
		for (Job job : schedule) {
			data.add(new Triple<String, Tick, Tick>(job.getProductAgentName(), job.getStartTime(), job.getDue()));
		}
		return data;
	}

	/**
	 * Get the history of the equiplet, the jobs that where executed
	 * 
	 * @return a list of jobs
	 */
	@Override
	public List<Triple<String, Tick, Tick>> getHistory() {
		List<Triple<String, Tick, Tick>> data = new ArrayList<Triple<String, Tick, Tick>>();
		for (Job job : history) {
			data.add(new Triple<String, Tick, Tick>(job.getProductAgentName(), job.getStartTime(), job.getDue()));
		}
		return data;
	}

	/**
	 * Information for updating the gui Tuple < name of equiplet, position,
	 * services, Tuple < state, waiting, scheduled, executed > >
	 * 
	 * @return information
	 */
	@Override
	public Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>> getUpdateState() {
		List<String> services = new ArrayList<String>();
		for (Capability capability : capabilities) {
			services.add(capability.getService());
		}
		Tuple<String, Integer, Integer, Integer> info = new Tuple<String, Integer, Integer, Integer>(getEquipletState().toString() + (reconfiguring ? " reconfiguring" : ""), getWaiting(), getScheduled(), getExecuted());
		return new Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>(name, getPosition(), services, info);
	}

	/**
	 * @return the remaining process time of the job
	 */
	@Override
	public Tick getRemainingTime() {
		return timeRemaining;
	}

	@Override
	public void kill() {
		// let the gc do his job
	}

}
