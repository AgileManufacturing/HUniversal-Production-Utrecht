package MAS.simulation.mas.equiplet;

import jade.core.AID;

import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.PriorityBlockingQueue;

import MAS.equiplet.Capability;
import MAS.equiplet.EquipletState;
import MAS.equiplet.Job;
import MAS.simulation.simulation.ISimulation;
import MAS.simulation.util.Settings;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.MasConfiguration;
import MAS.util.Tick;
import MAS.util.Tuple;

public class EquipletSimAgentDealWithItTemporyName extends EquipletSimAgent implements IEquipletSim {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Queue<Job> productQueue;

	public EquipletSimAgentDealWithItTemporyName(ISimulation simulation, Position position, List<Capability> capabilities) {
		super(simulation, position, capabilities);

		productQueue = new PriorityBlockingQueue<>(Settings.QUEUE_CAPACITY, new Comparator<Job>() {
			@Override
			public int compare(Job o1, Job o2) {
				return o1.getDeadline().equals(o2.getDeadline()) ? (o1.getIndex() < o1.getIndex() ? -1 : o1.getIndex() == o1.getIndex() ? 0 : 1)
						: o1.getDeadline().compareTo(o2.getDeadline());
			}
		});
	}

	/**
	 * Get the number of job that are scheduled for the equiplet
	 * 
	 * @return number of jobs
	 */
	protected synchronized int getScheduled() {
		return productQueue.size();
	}

	/**
	 * Get the number of jobs waiting to be executed i.e. ready for execution A
	 * job is ready for executing when the product arrived by the equiplet
	 * 
	 * @return number of jobs ready to executed
	 */
	protected synchronized int getWaiting() {
		return productQueue.size();
	}

	@Override
	protected Job jobReady() {
		return productQueue.poll();

	}

	@Override
	protected synchronized boolean isCapable(String service, Map<String, Object> criteria) {
		return productQueue.size() < Settings.QUEUE_CAPACITY && super.isCapable(service, criteria);
	}

	/**
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	@Override
	public synchronized double load(Tick time, Tick window) {
		return (Settings.QUEUE_CAPACITY * 1.0 - productQueue.size()) / Settings.QUEUE_CAPACITY;
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
	protected synchronized void notifyProductArrived(AID product, Tick time) {
		// execute the first job in the schedule if the job is ready
		if (state == EquipletState.IDLE && productQueue.size() > 0) {
			// begin with executing job that arrived
			Job ready = jobReady();
			executeJob(time, ready);
		} else if (state == EquipletState.ERROR && !isExecuting() && productQueue.size() > 0) {
			// Equiplet is still broken, but as soon as this is repaired it will execute the first job in the schedule
			System.out.printf("EA:%s product %s going to be executed after repair\n", getLocalName(), product.getLocalName());
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", getLocalName(), product);
		}
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
	@Override
	protected synchronized boolean schedule(AID product, int index, Tick start, Tick deadline, String service, Map<String, Object> criteria) {
		System.out.printf("EA:%s schedule a job \n", getLocalName());
		// do not schedule a job when going to be reconfigured
		// check if the equiplet still capable
		if (!isCapable(service, criteria) || reconfiguring || productQueue.size() >= Settings.QUEUE_CAPACITY) {
			return false;
		}

		Tick duration = estimateService(service);
		System.out.printf("EA:%s schedule [product=%s, start=%s, duration=%s, deadline=%s, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start, duration, deadline, service, criteria);

		return productQueue.add(new Job(index, product, service, criteria, start, deadline, deadline));
	}

	/**
	 * Schedule multiple jobs for one equiplet
	 * A request consists of a list of product steps, with a tuple of <
	 * production step index, a Pair of <start time, deadline>, service, and
	 * criteria >
	 * 
	 * @param product
	 *            agent
	 * @param requests
	 *            list of product step request of multiple product steps
	 * @return true if schedule succeeded
	 */
	@Override
	protected synchronized boolean schedule(AID product, List<Tuple<Integer, Pair<Tick, Tick>, String, Map<String, Object>>> requests) {
		System.out.printf("EA:%s schedule multiple jobs\n", getLocalName());
		boolean succeeded = true;
		for (Tuple<Integer, Pair<Tick, Tick>, String, Map<String, Object>> request : requests) {
			int index = request.first;
			Tick start = request.second.first;
			Tick deadline = request.second.second;
			String service = request.third;
			Map<String, Object> criteria = request.fourth;
			succeeded &= schedule(product, index, start, deadline, service, criteria);
		}

		System.out.println("succeeded  " + succeeded);
		return succeeded;
	}

	@Override
	public String toString() {
		return String.format("%s:[state=%s, capabilities=%s, executing=%s, scheduled=%d, history=%d, queue=%s]", getLocalName(), state, capabilities, (state == EquipletState.IDLE ? "null"
				: executing), productQueue.size(), history.size(), productQueue);
	}

}
