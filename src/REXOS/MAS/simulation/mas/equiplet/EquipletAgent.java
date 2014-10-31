package MAS.simulation.mas.equiplet;

import jade.core.AID;
import jade.core.Agent;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPANames;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import org.json.JSONException;

import MAS.simulation.mas.product.ProductStep;
import MAS.simulation.util.Ontology;
import MAS.simulation.util.Pair;
import MAS.simulation.util.Parser;
import MAS.simulation.util.Position;
import MAS.simulation.util.Settings;
import MAS.simulation.util.Tick;
import MAS.simulation.util.Triple;
import MAS.simulation.util.Tuple;

public class EquipletAgent extends Agent {

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
	protected boolean reconfigure;
	protected Job executing;
	protected TreeSet<Job> history;
	protected Map<String, Tick> productionTimes;

	/**
	 * Equiplet agent startup
	 */
	@Override
	public void setup() {
		Object[] args = getArguments();

		if (args != null && args.length > 0) {
			try {
				Pair<Position, List<Capability>> configuration = Parser.parseEquipletConfiguration(args[0].toString());

				init(configuration.first, configuration.second);
				System.out.printf("EA:%s initialize [pos=%s, capabilties=%s]\n", getLocalName(), position, capabilities);

				register();

				addBehaviour(new EquipletListenerBehaviour(this));

			} catch (JSONException e) {
				System.err.printf("EA:%s failed to parse the arguments\n", getLocalName());
				System.err.printf("EA:%s %s", getLocalName(), e.getMessage());
			}
		} else {
			System.err.printf("EA:%s Failed to receive correct arguments\n", getLocalName());
		}
	}

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
		this.reconfigure = false;
		this.executing = null;
		this.schedule = new TreeSet<>();
		this.history = new TreeSet<>();

		this.productionTimes = new HashMap<>();
		for (Capability capability : capabilities) {
			productionTimes.put(capability.getService(), capability.getDuration());
		}
	}

	/**
	 * Register the equiplet services by the Directory Facilitator Agent
	 */
	protected void register() {
		DFAgentDescription dfAgentDescription = new DFAgentDescription();
		dfAgentDescription.setName(getAID());
		for (Capability capability : capabilities) {
			ServiceDescription serviceDescription = new ServiceDescription();
			serviceDescription.setName(capability.getService());
			serviceDescription.setType(Ontology.SERVICE_SEARCH_TYPE);
			serviceDescription.addOntologies(Ontology.GRID_ONTOLOGY);
			serviceDescription.addLanguages(FIPANames.ContentLanguage.FIPA_SL);
			dfAgentDescription.addServices(serviceDescription);
		}
		try {
			DFService.register(this, dfAgentDescription);
		} catch (FIPAException fe) {
			System.err.printf("EA:%s Failed to register services\n", getLocalName());
			fe.printStackTrace();
		}
	}

	/**
	 * Deregister the equiplet services by the Directory Facilitator Agent
	 */
	public void deregister() {
		try {
			DFService.deregister(this);
		} catch (FIPAException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Euiplet agent clean-up operations
	 */
	@Override
	protected void takeDown() {
		deregister();
		System.out.printf("EA:%s terminating\n", getLocalName());
	}

	/**
	 * @return the position of the equiplet
	 */
	protected Position getPosition() {
		return position;
	}

	/**
	 * @return the state of the equiplet
	 */
	protected EquipletState getEquipletState() {
		return state;
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
	protected int getExecuted() {
		return history.size();
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
	 * Checks whether a job is ready for execution
	 * TODO check not only the first in the schedule but also after if job can
	 * be executed earlier than planned, which increases complexity
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
			if (counter >= Settings.QUEUE_JUMP) {
				break;
			}
		}
		return null;
		// return !schedule.isEmpty() && schedule.first().isReady() ? schedule.first() : null;
		// return !schedule.isEmpty() && schedule.first().isReady();
	}

	/**
	 * check whether the equiplet can execute a list of product steps within a
	 * time frame
	 * this returns the list of services that can be performed @see {@code isCapable}, with the estimate duration of the service and a list
	 * of possible time frames
	 * the time frame is a list of times from which the equiplet is free until
	 * the equiplet busy again where the deadline is the time till when is
	 * looked
	 * 
	 * @param time
	 *            of the first possibility to perform the product steps
	 * @param deadline
	 *            of the product steps
	 * @param productSteps
	 *            the list of product steps to be performed
	 * @return
	 */
	public synchronized List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> canExecute(Tick time, Tick deadline, List<ProductStep> productSteps) {
		// answer :: List of services < index in production path, estimate production time, List of from and until time when possible>
		List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> answer = new ArrayList<>();

		for (ProductStep productStep : productSteps) {
			if (isCapable(productStep.getService(), productStep.getCriteria())) {
				int index = productStep.getIndex();
				Tick duration = estimateService(productStep.getService());
				List<Pair<Tick, Tick>> available = available(time, duration, deadline);
				answer.add(new Triple<Integer, Tick, List<Pair<Tick, Tick>>>(index, duration, available));
			}
		}

		return answer;
	}

	/**
	 * check whether the equiplet is capable to perform a service with certain
	 * criteria
	 * 
	 * @param service
	 * @param criteria
	 * @return whether the equiplet is capable to perform the service
	 */
	protected synchronized boolean isCapable(String service, Map<String, Object> criteria) {
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
	 * the first possible time there is enough room in the schedule to perform a
	 * service
	 * load = 1 - Sr / Sw
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

		// not availale when going to be reconfigured
		if (reconfigure) {
			return available;
		}

		Tick start = time;
		if (isExecuting()) {
			start = start.max(executing.getDue());
		}

		if (schedule.isEmpty()) {
			available.add(new Pair<Tick, Tick>(start, deadline));
		} else {
			Iterator<Job> it = schedule.iterator();
			while (it.hasNext()) {
				Job job = it.next();

				if (job.getStartTime().greaterOrEqualThan(start)) {

					if (job.getStartTime().minus(start).greaterThan(duration)) {

						if (job.getStartTime().lessThan(deadline)) {
							available.add(new Pair<Tick, Tick>(start, job.getStartTime()));
							start = job.getDue();
						} else {
							available.add(new Pair<Tick, Tick>(start, deadline));
							break;
						}
					} else {
						start = job.getDue();
					}
				} else {
					start = job.getDue().max(start);
				}

				if (!it.hasNext() && start.lessThan(deadline)) {
					available.add(new Pair<Tick, Tick>(start, deadline));
				}
			}
		}

		return available;
	}

	/**
	 * calculate the load of the equiplet from a certain time with a window
	 * load = 1 - Sr / Sw
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	protected synchronized double load(Tick time, Tick window) {
		Tick sum = new Tick(0);

		for (Job job : schedule) {
			if (job.getStartTime().greaterOrEqualThan(time) && job.getStartTime().lessOrEqualThan(time.add(window)) || job.getDue().greaterThan(time)
					&& job.getDue().lessOrEqualThan(time.add(window))) {
				sum = sum.add(job.getDue().min(time.add(window)).minus(job.getStartTime().max(time)));
			}

			if (job.getStartTime().greaterThan(time.add(window))) {
				break;
			}
		}

		// System.out.println("EA:" + getLocalName() + " load= 1 - " + sum + " / " + window + " = " + (1 - sum / window) + " in " + schedule);

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
	protected synchronized double loadHistory(Tick time, Tick window) {
		Tick sum = new Tick(1); // dirty fix

		Iterator<Job> iterator = history.descendingIterator();
		while (iterator.hasNext()) {
			Job job = iterator.next();
			if (job.getStartTime().greaterOrEqualThan(time) && job.getStartTime().lessOrEqualThan(time.add(window)) || job.getDue().greaterThan(time)
					&& job.getDue().lessOrEqualThan(time.add(window))) {
				sum = sum.add(job.getDue().min(time.add(window)).minus(job.getStartTime().max(time)));
			} else if (job.getDue().lessThan(time)) {
				// the jobs are outside the scope of the load window
				// break;
			}
		}

		// double precision error, dirty fix, can use BigDecimal although performance
		sum = new Tick(Math.round(sum.doubleValue() * 100000000) / 100000000);

		return 1 - sum.div(window).doubleValue();
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
	protected synchronized boolean schedule(AID product, int index, Tick start, Tick deadline, String service, Map<String, Object> criteria) {
		// do not schedule a job when going to be reconfigured
		if (reconfigure) {
			throw new IllegalArgumentException("not able to schedule job when reconfiguring" );
			// return false;
		}

		Tick duration = estimateService(service);
		System.out.printf("EA:%s schedule [product=%s, start=%s, duration=%s, deadline=%s, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start, duration, deadline, service, criteria);

		if (schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))).isEmpty()) {
			Job job = new Job(index, product, service, criteria, start, start.add(duration), deadline);
			return schedule.add(job);
		} else {
			// this shouldn't yet occur (not in the simulation), a equiplet should never give a product the available time which cannot be scheduled
			System.err.println("\n----------\nstart=" + start + ", due=" + start.add(duration) + "\nSCHEDULE:\n" + schedule + "\n\n");
			throw new IllegalArgumentException("overlap schedule ");
		}
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
	protected synchronized boolean schedule(AID product, List<Tuple<Integer, Pair<Tick, Tick>, String, Map<String, Object>>> requests) {
		// do not schedule a job when going to be reconfigured
		if (reconfigure) {

			throw new IllegalArgumentException("not able to schedule job when reconfiguring" );
			//return false;
		}

		List<Job> possible = new ArrayList<Job>();
		for (Tuple<Integer, Pair<Tick, Tick>, String, Map<String, Object>> data : requests) {
			int index = data.first;
			Tick start = data.second.first;
			Tick deadline = data.second.second;
			String service = data.third;
			Map<String, Object> criteria = data.fourth;

			// check if the equiplet still capable
			if (isCapable(service, criteria)) {
				// if the request start is after the due time of the executing job
				if (!isExecuting() || start.greaterOrEqualThan(executing.getDue())) {
					Tick duration = estimateService(service);

					System.out.printf("EA:%s schedule [product=%s, index=%d, start=%s, duration=%s, deadline=%s, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), index, start, duration, deadline, service, criteria);

					if (schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))).isEmpty()) {
						Job job = new Job(index, product, service, criteria, start, start.add(duration), deadline);

						possible.add(job);
					} else {
						// this shouldn't yet occur (not in the simulation), a equiplet should never give a product the available time which cannot be scheduled
						System.err.println("\n----------\nstart=" + start + ", due=" + start.add(duration) + "\nSCHEDULE:\n" + schedule + "\n\n");
						throw new IllegalArgumentException("overlap schedule: " + schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))));
					}
				} else {
					// this shouldn't occur, TODO remove exception and log the error and return false, send fail message which must be handled!
					System.err.printf("EA:%s failed to schedule: request to schedule job starting at %s when still executing %s other job.\n", getLocalName(), start, executing);
					throw new IllegalArgumentException("request to schedule job starting at " + start + " when still executing " + executing + " other job");
				}
			} else {
				// this shouldn't occur, TODO remove exception nd log the error and return false, send fail message which must be handled!
				// maybe possible when reconfigured??? probable not.
				System.err.printf("EA:%s failed to schedule: not capable to execute service %s with criteria %s.\n", getLocalName(), service, criteria);
				throw new IllegalArgumentException("not capable to execute service " + service + " with criteria " + criteria);
			}
		}

		/*
		 * TODO check correctness of this before adding
		 * it could be possible that job
		 * int job;
		 * boolean successful = true;
		 * for (job = 0; job < possible.size(); job++) {
		 * successful = schedule.add(possible.get(job));
		 * if (!successful) {
		 * break;
		 * }
		 * }
		 * 
		 * // rollback added jobs, although first checked if it is possible,
		 * there is still gone something wrong
		 * if (!successful) {
		 * for (int i = job; i >= 0; i--) {
		 * schedule.remove(possible.get(i));
		 * }
		 * }
		 * return successful;
		 */

		for (Job job : possible) {
			schedule.add(job);
		}

		return true;

	}

	protected synchronized void updateSchedule() {
		// After being broken down and repaired the jobs in the schedule can be delayed.
		// The executing job is updated with the new due time
		// The scheduled jobs should have, depending if the jobs are continuous scheduled, a new start time added with the delay
		// Although the jobs doesn't have to be continuous scheduled, the start time depends on the due date of the previous job
		// The new start time is the max of ( due time of previous job, or the original start time)
		if (isExecuting()) {
			Tick dueTime = executing.getDue();
			for (Job job : schedule) {
				if (dueTime.greaterThan(job.getStartTime())) {
					job.updateStartTime(dueTime);
					dueTime = job.getDue();
				} else {
					// no change in start time, so continuing would not change the schedule
					break;
				}
			}
		}
	}

	@Override
	public String toString() {
//		return String.format("%s:[state=%s, capabilities=%s, executing=%s, scheduled=%d, waiting=%d, history=%d, schedule=%s]", getLocalName(), state, capabilities,  (state == EquipletState.IDLE ? "null"
//				: executing), schedule.size(), getWaiting(), history.size(), "schedule");
		return String.format("%s:[state=%s, capabilities=%s, executing=%s, scheduled=%d, waiting=%d, history=%d, schedule=%s]", getLocalName(), state, capabilities,  (executing == null ? "null"
				: executing), schedule.size(), getWaiting(), history.size(), "schedule");
	}

	/**
	 * Start with executing the first job in the schedule Note: that the first
	 * job in the schedule need to be ready
	 * TODO fix that the job can be performed earlier that scheduled.
	 * 
	 * @param start
	 *            time of the job
	 * @param job
	 *            to be executed
	 */
	protected synchronized void executeJob(Tick time, Job job) {
		System.out.println(" Execute Job " + time + " job = " + job);
		state = EquipletState.BUSY;
		System.out.println(" Execute Job " + time + " job = " + job);
		executing = job;

		executing.updateStartTime(time);
		System.out.printf("EA:%s starts at %s with executing job: %s\n", getLocalName(), time, executing);

		informProductProcessing(executing.getProductAgent(), time, executing.getIndex());

		execute(executing);
	}

	/**
	 * The actual start of executing a job
	 * 
	 * @param job
	 */
	protected void execute(Job job) {

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
	protected synchronized void notifyProductArrived(AID product, Tick time) {
		// check if it is needed to also check the criteria
		// TODO possible service not necessary, if making the constraint that a
		// product can only have one job ready by an equiplet

		// check if the job arrived is the first in the schedule, than just execute.
		// otherwise, if the first job is too late the arrived job is second, this results in that the already too late (first) job is
		// if there are jobs between the first job and arrived job, swapping of jobs can take place if all deadlines are met.
		Job ready = null;
		int index = 0;
		for (Job job : schedule) {
			if (job.getProductAgent().equals(product)) {
				job.setReady();

				// job can only be executed earlier than planned if equiplet is idle
				if (state == EquipletState.IDLE) {
					if (index == 0) {
						ready = schedule.pollFirst();
					} else if (index == 1) {
						// swapping jobs
						ready = job;
						schedule.remove(job);
						Job first = schedule.first();

						System.out.printf("EA:%s swapping job %s that is ready with %s\n", getLocalName(), ready, first);
						System.out.printf("EA:%s equiplet=%s\n", getLocalName(), this);

						// first.updateStartTime(time.add(job.getDuration()));
						first.updateStartTime(ready.getStartTime());

						System.out.printf("EA:%s equiplet=%s\n", getLocalName(), this);
					}
				}

				break;
			}
			index++;
		}

		// TODO combine the set ready loop above with the possibility to execute
		// a job that is later in the schedule but can already be performed

		// execute the first job in the schedule if the job is ready
		if (state == EquipletState.IDLE && ready != null) { // && jobReady()) {
			// begin with executing job that arrived
			executeJob(time, ready);
		} else if (state == EquipletState.ERROR && !isExecuting() && jobReady() != null) {
			// Equiplet is still broken, but as soon as this is repaired it will execute the first job in the schedule
			System.out.printf("EA:%s product %s going to be executed after repair\n", getLocalName(), product.getLocalName());
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", getLocalName(), product);
		}
	}

	/**
	 * Inform the product agent that the job started to be executed is for the
	 * product agent
	 * Thereafter wait for confirmation of the message is received
	 * 
	 * @param product
	 *            agent address
	 */
	protected void informProductProcessing(AID product, Tick time, int intdex) {
		try {
			// send product agent information about going to process product
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(product);
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_PROCESSING);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_PROCESSING + System.currentTimeMillis());
			message.setContent(Parser.parseProductProcessing(time, intdex));
			send(message);

			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template, Settings.COMMUNICATION_TIMEOUT);

			if (reply == null || !Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product processing.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product started to be processed.\n", getLocalName(), executing.getProductAgentName());
			System.err.printf("EA:%s %s\n", getLocalName(), e.getMessage());
		}
	}

	/**
	 * Inform the product agent the job executed form him is finished
	 * 
	 * @param product
	 *            agents address
	 */
	protected void informProductStepFinished(AID product, Tick time, int intdex) {
		try {
			// send product agent information about going to process product
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(product);
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_FINISHED + System.currentTimeMillis());
			message.setContent(Parser.parseProductFinished(time, intdex));
			send(message);

			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template, Settings.COMMUNICATION_TIMEOUT);

			if (reply == null || !Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product %s his product step finished. %s\n", getLocalName(), product, reply);
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product step is finished.\n", getLocalName(), (executing != null ? executing.getProductAgentName()
					: "null"));
			System.err.printf("EA:%s %s\n", getLocalName(), e.getMessage());
		}
	}
}
