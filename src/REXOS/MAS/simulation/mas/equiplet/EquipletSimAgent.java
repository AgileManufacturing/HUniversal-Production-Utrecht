package MAS.simulation.mas.equiplet;

import jade.core.AID;
import jade.domain.DFService;
import jade.domain.FIPAException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.json.JSONException;

import MAS.equiplet.Capability;
import MAS.equiplet.EquipletAgent;
import MAS.equiplet.EquipletState;
import MAS.equiplet.Job;
import MAS.simulation.simulation.ISimulation;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;
import MAS.util.Tuple;

public class EquipletSimAgent extends EquipletAgent implements IEquipletSim {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

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

	private ISimulation simulation;

	public EquipletSimAgent(ISimulation simulation, Position position, List<Capability> capabilities) {
		try {
			Object[] args = new Object[] { Parser.parseEquipletConfiguration(position, capabilities) };
			setArguments(args);
		} catch (JSONException e) {
			System.err.printf("EA: failed to create equiplet: %s.\n", e.getMessage());
		}

		this.simulation = simulation;
		lastStatisticsUpdate = new Tick(0);
		statistics = new Triple<Tick, Tick, Tick>(new Tick(0), new Tick(0), new Tick(0));
		scheduleLatency = new HashMap<Tick, Tick>();
	}

	@Override
	public void kill() {
		try {
			// deregister equiplet by the df
			DFService.deregister(this);
		} catch (FIPAException e) {
			System.err.println("failed to deregister equiplet");
			e.printStackTrace();
		} finally {
			super.doDelete();
		}
	}

	@Override
	public Position getPosition() {
		return position;
	}

	@Override
	public EquipletState getEquipletState() {
		return state;
	}

	@Override
	public List<Capability> getCapabilities() {
		return capabilities;
	}

	/**
	 * 
	 * @param capabilities
	 */
	@Override
	public void reconfigureStart(List<Capability> capabilities) {
		System.out.printf("EA:%s reconfigure with capabilities %s to new capabilties %s \n", getLocalName(), this.capabilities, capabilities);
		reconfigure = true;
		deregister();
		this.capabilities = capabilities;
	}

	/**
	 * 
	 */
	@Override
	public void reconfigureFinished() {
		System.out.printf("EA:%s reconfigure finished he has new capabilties %s \n", getLocalName(), capabilities);
		if (schedule.isEmpty()) {
			reconfigure = false;
			for (Capability capability : capabilities) {
				productionTimes.put(capability.getService(), capability.getDuration());
			}
			register();
			state = EquipletState.IDLE;
		} else {
			throw new IllegalArgumentException("Equiplet has not an empty schedule while being reconfigured");
		}
	}

	@Override
	public double load(Tick time, Tick window) {
		return super.load(time, window);
	}

	@Override
	public double loadHistory(Tick time, Tick window) {
		return super.loadHistory(time, window);
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
	 * @return the difference between the start time and the planned start time
	 */
	@Override
	public Map<Tick, Tick> getLatency() {
		return scheduleLatency;
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
		Tuple<String, Integer, Integer, Integer> info = new Tuple<String, Integer, Integer, Integer>(getEquipletState().toString() + (reconfigure ? " reconfiguring" : ""), getWaiting(), getScheduled(), getExecuted());
		return new Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>(getLocalName(), getPosition(), services, info);
	}

	/**
	 * @return the remaining process time of the job
	 */
	@Override
	public Tick getRemainingTime() {
		return timeRemaining;
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

	/**
	 * Validate the schedule, this function should TODO check if there are no
	 * jobs that should be performed before a time, but has not. Note: this
	 * should be return true also if the
	 * job first becomes ready but a job is being performed before he was
	 * scheduled for performence improvement.
	 * 
	 * @param time
	 * @return is the schedule validates
	 */
	@Deprecated
	@SuppressWarnings("unused")
	private boolean validateSchedule(Tick time) {
		return true;
	}

	/**
	 * Start with executing the first job in the schedule Note: that the first
	 * job in the schedule need to be ready TODO fix that the job can be
	 * performed earlier that scheduled.
	 * 
	 * @param start
	 *            time of the job
	 * @param job
	 *            to be executed
	 */
	@Override
	protected synchronized void executeJob(Tick time, Job job) {
		state = EquipletState.BUSY;
		executing = job;

		Tick latency = time.minus(executing.getStartTime());
		scheduleLatency.put(time, latency);
		executing.updateStartTime(time);
		System.out.printf("EA:%s starts at %s (%s from scheduled time) with executing job: %s\n", getLocalName(), time, latency, executing);

		informProductProcessing(executing.getProductAgent(), time, executing.getIndex());
		execute(executing);
	}

	@Override
	protected void execute(Job job) {
		// do nothing to execute job
	}

	/**
	 * Notify a product is arrived by the equiplet and is ready to be let the
	 * equiplet execute his product step
	 * 
	 * @param product
	 *            name of the product agent
	 * @param time
	 *            of the the product arrival
	 */
	@Override
	protected void notifyProductArrived(AID product, Tick time) {
		historyUpdate(time);
		super.notifyProductArrived(product, time);
	}

	/**
	 * Notify the executing job is finished
	 * 
	 * @param time
	 *            the job finished
	 */
	public void notifyJobFinished(Tick time) {
		if (state == EquipletState.ERROR) {
			// the equiplet should have finished the job, but was broken down in the meantime
			// the equiplet has still a remaining time to continue after the equiplet is repaired
			state = EquipletState.ERROR_FINISHED;
			timeRemaining = time.minus(timeBreakdown);
			executing.updateDueTime(executing.getDue().add(timeRemaining));
			System.out.printf("EA:%s job %s should finished but delayed by breakdown, should still %s be executed after being repaired.\n", getLocalName(), executing, timeRemaining);
		} else if (state == EquipletState.ERROR_REPAIRED) {
			// the equiplet should have finished with the job, but was broken down in the meantime,
			// the equiplet continues with executing the job
			System.out.printf("EA:%s job %s should finished but delayed by breakdown, should still %s be executed.\n", getLocalName(), executing, timeRemaining);
			state = EquipletState.BUSY;
		} else if (state == EquipletState.BUSY) {
			// executing of the job is really finished and will continue with the next job if possible
			executing.updateDueTime(time);
			history.add(executing);
			historyUpdate(time);

			System.out.printf("EA:%s finished with job: %s\n", getLocalName(), executing);

			AID finishedProduct = executing.getProductAgent();
			int index = executing.getIndex();

			Job ready = jobReady();
			if (ready != null) {
				// if (!schedule.isEmpty() && jobReady()) {
				schedule.remove(ready);
				executeJob(time, ready);
			} else if (reconfigure && schedule.isEmpty()) {
				state = EquipletState.RECONFIG;
				if (simulation == null) {
					throw new IllegalArgumentException("FUCK sim");
				}
				simulation.notifyReconfigReady(getLocalName());
			} else {
				state = EquipletState.IDLE;
				executing = null;
			}

			// note that the inform processing is done before inform finished
			// this is because the simulation can delete the product agent (if chosen to do so for performance improvement)
			// therefore there is no guarantee that informing the product is a blocking as the acknowledge is send before notifying the simulation
			informProductStepFinished(finishedProduct, time, index);
		} else {
			throw new IllegalArgumentException("EQUIPLET: notify job finished not given in correct state: " + state);
		}
	}

	/**
	 * Notify the equiplet is broken down A constraint is that the equiplet can
	 * only be idle or busy when this can happen
	 * 
	 * @param time
	 *            of the breakdown
	 */
	public void notifyBreakdown(Tick time) {
		if (state == EquipletState.ERROR || state == EquipletState.ERROR_READY || state == EquipletState.ERROR_FINISHED || state == EquipletState.RECONFIG) {
			throw new IllegalArgumentException("EQUIPLET: notify breakdown not given in correct state: " + state);
		}

		if (state == EquipletState.ERROR_REPAIRED) {
			timeBreakdown = time.minus(timeRemaining);
			System.out.printf("EA:%s is broken down at %s, substracting the previous broken time %s\n", getLocalName(), time, timeRemaining);
		} else {
			timeBreakdown = time;
			System.out.printf("EA:%s is broken down at %s\n", getLocalName(), time);
		}
		historyUpdate(time);
		state = EquipletState.ERROR;
	}

	/**
	 * The notify that the equiplet is repaired if the equiplet has finished
	 * during the repair, remember the time the equiplet has broken
	 * 
	 * @param time
	 *            of repair
	 */
	public void notifyRepaired(Tick time) {
		if (state == EquipletState.IDLE || state == EquipletState.BUSY || state == EquipletState.ERROR_REPAIRED || state == EquipletState.RECONFIG) {
			throw new IllegalArgumentException("EQUIPLET: notify breakdown not given in correct state: " + state);
		}
		historyUpdate(time);

		if (state == EquipletState.ERROR_FINISHED) {
			// the equiplet has already a finished event received, but is now repaired and can continue with the job
			state = EquipletState.BUSY;
			// executing.updateDueTime(executing.getDue() + (time - timeBreakdown));
			System.out.printf("EA:%s is repaired at %s and continue with job %s, with %s time remaining.\n", getLocalName(), time, executing, timeRemaining);
		} else if (state == EquipletState.ERROR_READY) {
			// in the time the equiplet was broken there is a product arrived that can be executed
			Job ready = jobReady();
			schedule.remove(ready);
			executeJob(time, ready);
		} else if (isExecuting()) {
			// the equiplet is executing a job and is repaired, but waits until a job finished event is received
			state = EquipletState.ERROR_REPAIRED;
			timeRemaining = time.minus(timeBreakdown);
			executing.updateDueTime(executing.getDue().add(timeRemaining));
			System.out.printf("EA:%s is repaired at %s and continue with job %s. The equiplet was %s broken, which is still remaining.\n", getLocalName(), time, executing, timeRemaining);
			// } else if (jobReady()) {
			// when the equiplet was in the error state there became a job ready which arrived before the breakdown
			// not sure if this could happen
			// System.out.println("EQUIPLET ERROR? " + schedule);
			// executeJob(time);
			// System.out.printf("EA:%s is repaired at %s and detect that a job has became ready: %s \n", getLocalName(), time, executing);
		} else {
			// the equiplet has nothing to do and goes into IDLE state
			System.out.printf("EA:%s is repaired at %s \n", getLocalName(), time);
			state = EquipletState.IDLE;
		}

		updateSchedule();
	}
}
