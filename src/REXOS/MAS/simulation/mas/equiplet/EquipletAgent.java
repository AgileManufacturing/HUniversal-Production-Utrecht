package simulation.mas.equiplet;

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
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import org.json.JSONException;

import simulation.mas.product.ProductStep;
import simulation.util.Capability;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.Ontology;
import simulation.util.Triple;
import simulation.util.Tuple;

public class EquipletAgent extends Agent {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private static final double SAFETY_FACTOR = 1;
	private Position position;
	private List<Capability> capabilities;
	private Map<String, Double> productionTimes;

	// Equiplet state
	private EquipletState state;
	private Job executing;

	private TreeSet<Job> schedule;
	private List<Job> history;

	// Simulation variables
	private double timeBreakdown;
	private double timeRemaining;

	/**
	 * @var statistics Statistics contains the time the equiplet is in one of the states <BUSY, IDLE, ERROR> The states ERROR_READY and ERROR_FINISHED are counted as ERROR and
	 *      ERROR_REPAIRED as BUSY
	 */
	private Triple<Double, Double, Double> statistics;

	/**
	 * @var lastHistoryUpdate The last time the statistics is update to calculate the elapsed time between state changes
	 */
	private double lastHistoryUpdate;

	/**
	 * @var scheduleLatency A list of differences between the time and the time scheduled
	 */
	private Map<Double, Double> scheduleLatency;

	public EquipletAgent(Position position, List<Capability> capabilities, Map<String, Double> productionTimes) {
		try {
			Object[] args = new Object[] { Parser.parseEquipletConfiguration(position, capabilities, productionTimes) };
			setArguments(args);
		} catch (JSONException e) {
			System.err.printf("EA: failed to create equiplet: %s.\n", e.getMessage());
		}
	}

	public void setup() {
		Object[] args = getArguments();

		if (args != null && args.length > 0) {
			try {
				Triple<Position, List<Capability>, Map<String, Double>> configuration = Parser.parseEquipletConfiguration(args[0].toString());

				this.position = configuration.first;
				this.capabilities = configuration.second;
				this.productionTimes = configuration.third;

				System.out.printf("EA:%s initialize [pos=%s, capabilties=%s, production times=%s]\n", getLocalName(), position, capabilities, productionTimes);

				this.state = EquipletState.IDLE;
				this.executing = null;

				this.schedule = new TreeSet<>();
				this.history = new ArrayList<>();

				this.timeBreakdown = -1;
				this.timeRemaining = -1;

				this.lastHistoryUpdate = 0;
				this.statistics = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);
				this.scheduleLatency = new HashMap<Double, Double>();

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

	private void register() {
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
	 * Put agent clean-up operations here
	 */
	protected void takeDown() {
		System.out.printf("EA:%s terminating\n", getLocalName());
	}

	/**
	 * Get the state of the equiplet
	 * 
	 * @return the equiplet state
	 */
	public EquipletState getEquipletState() {
		return state;
	}

	protected Pair<String, Double> getExecuting() {
		return new Pair<String, Double>(executing.getProductAgentName(), executing.getStartTime());
	}

	@Deprecated
	protected List<String> getServices() {
		ArrayList<String> services = new ArrayList<>();
		for (Capability capability : capabilities) {
			services.add(capability.getService());
		}
		return services;
	}

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

	protected boolean isCapable(String service, Map<String, Object> criteria) {
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

	@Deprecated
	protected boolean providesService(String service) {
		return productionTimes.containsKey(service);
	}

	public Position getPosition() {
		return position;
	}

	/**
	 * Get the number of jobs waiting to be executed i.e. ready for execution A job is ready for executing when the product arrived by the equiplet
	 * 
	 * @return number of jobs ready to executed
	 */
	protected int getWaiting() {
		int waiting = 0;
		for (Job job : schedule) {
			if (job.isReady()) {
				waiting++;
			}
		}
		return waiting;
	}

	/**
	 * Get the number of job that are scheduled for the equiplet
	 * 
	 * @return number of jobs
	 */
	protected int getScheduled() {
		return schedule.size();
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
	 * @return the remaining process time of the job
	 */
	public double getRemainingTime() {
		return timeBreakdown;
	}

	/**
	 * update the statistics This method should be called after each state change
	 * 
	 * @param time
	 */
	protected void historyUpdate(double time) {
		double elapsed = time - lastHistoryUpdate;
		lastHistoryUpdate = time;
		if (state == EquipletState.BUSY) {
			statistics.first += elapsed;
		} else if (state == EquipletState.IDLE) {
			statistics.second += elapsed;
		}  else if (state == EquipletState.ERROR) {
			statistics.third += elapsed;
		}
	}

	/**
	 * Retrieve the statistics of the equiplet
	 * 
	 * @param time
	 *            for last update to include time of question
	 * @return time the equiplet is <BUSY, IDLESY, ERROR>
	 */
	public Triple<Double, Double, Double> getStatistics(double time) {
		historyUpdate(time);
		return statistics;
	}

	public List<Triple<String, Double, Double>> getHistory() {
		List<Triple<String, Double, Double>> data = new ArrayList<Triple<String, Double, Double>>();
		for (Job job : history) {
			data.add(new Triple<String, Double, Double>(job.getProductAgentName(), job.getStartTime(), job.getDueTime()));
		}
		return data;
	}

	public List<Triple<String, Double, Double>> getSchedule() {
		List<Triple<String, Double, Double>> data = new ArrayList<Triple<String, Double, Double>>();
		if (isExecuting()) {
			data.add(new Triple<String, Double, Double>(executing.getProductAgentName(), executing.getStartTime(), executing.getDueTime()));
		}
		for (Job job : schedule) {
			data.add(new Triple<String, Double, Double>(job.getProductAgentName(), job.getStartTime(), job.getDueTime()));
		}
		return data;
	}

	public List<Triple<String, Double, Double>> getCompleteSchedule() {
		List<Triple<String, Double, Double>> data = new ArrayList<Triple<String, Double, Double>>();
		for (Job job : history) {
			data.add(new Triple<String, Double, Double>(job.getProductAgentName(), job.getStartTime(), job.getDueTime()));
		}
		if (isExecuting()) {
			data.add(new Triple<String, Double, Double>(executing.getProductAgentName(), executing.getStartTime(), executing.getDueTime()));
		}
		for (Job job : schedule) {
			data.add(new Triple<String, Double, Double>(job.getProductAgentName(), job.getStartTime(), job.getDueTime()));
		}
		return data;
	}

	public Map<Double, Double> getLatency() {
		return scheduleLatency;
	}

	/**
	 * schedule a job
	 * 
	 * @param start
	 *            time
	 * @param deadline
	 *            of job
	 * @param product
	 *            agent
	 * @param service
	 *            to be performed
	 * @param criteria
	 *            of the job
	 * @return if it succeeded to schedule the job
	 */
	@Deprecated
	protected boolean schedule(double start, double deadline, String product, String service, Map<String, Object> criteria) {
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
	 * schedule a job
	 * 
	 * @param start
	 *            time
	 * @param deadline
	 *            of job
	 * @param product
	 *            agent
	 * @param service
	 *            to be performed
	 * @param criteria
	 *            of the job
	 * @return if it succeeded to schedule the job
	 */
	protected boolean schedule(AID product, double start, double deadline, String service, Map<String, Object> criteria) {
		double duration = estimateService(service);
		System.out.printf("EA:%s schedule [product=%s, start=%.2f, duration=%.2f, deadline=%.2f, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start, duration, deadline, service, criteria);

		if (schedule.subSet(new Job(start, 0), true, new Job(start, duration), true).isEmpty()) {
			Job job = new Job(product, service, criteria, start, start + duration, deadline);
			schedule.add(job);
			return true;
		} else {
			return false;
		}
	}

	/**
	 * estimate the production time of a service
	 * 
	 * @param service
	 *            name
	 * @return production time
	 */
	private double estimateService(String service) {
		return productionTimes.get(service) * SAFETY_FACTOR;
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
	 * @return the available time
	 */
	/**
	 * 
	 * @param time
	 *            the first possible time from which to look
	 * @param duration
	 *            an estimate of time the equiplet is checked for availability
	 * @return a list of time it is possible to plan the duration TODO can only plan in end of schedule
	 */
	protected List<Pair<Double, Double>> available(double time, double duration, double deadline) {
		// TODO fix this so job can be scheduled in between jobs instead of
		// always behind the last
		List<Pair<Double, Double>> available = new ArrayList<Pair<Double, Double>>();
		if (schedule.size() > 0) {
			Job job = schedule.last();
			available.add(new Pair<Double, Double>(job.getDueTime(), deadline));
		} else if (isExecuting()) {
			available.add(new Pair<Double, Double>(executing.getDueTime(), deadline));
		} else {
			available.add(new Pair<Double, Double>(time, deadline));
		}
		return available;
	}

	@Override
	public String toString() {
		if (state == EquipletState.ERROR) {
			return String.format("%s:[state=%s, capabilities=%s, time of breakdown=%.2f, executing=%s, schedule=%d, waiting=%d, history=%d]", getLocalName(), state, capabilities, timeBreakdown, executing, schedule.size(), getWaiting(), history.size());
		} else {
			return String.format("%s:[state=%s, capabilities=%s, executing=%s, schedule=%d, waiting=%d, history=%d]", getLocalName(), state, capabilities, (state == EquipletState.IDLE ? "null"
					: executing), schedule.size(), getWaiting(), history.size());
		}
	}

	/**
	 * TODO RENAME function name The simulation need to check if the just scheduled product step is going to be executed Simulation check whether there is need for scheduling a job
	 * finished event
	 * 
	 * @param product
	 *            name
	 * @param service
	 * @param map
	 * @return is the job
	 */
	private boolean isExecutingStep(String product, String service, Map<String, Object> map) {
		// System.out.println(" isExecutingStep " + executing + " " + exe)
		return (executing != null && executing.getProductAgentName().equalsIgnoreCase(product) && executing.getService().equalsIgnoreCase(service) && executing.getCriteria().equals(map));
	}

	/**
	 * 
	 * @return true if the equiplet is busy executing a job
	 */
	protected boolean isExecuting() {
		return executing != null;
	}

	/**
	 * get the product agent for which the executing job is being performed
	 * 
	 * @return product agent
	 */
	public String getExecutingProduct() {
		return executing.getProductAgentName();
	}

	/**
	 * Checks whether a job is ready for execution TODO check not only the first in the schedule but also after if job can be executed earlier than planned, which increases
	 * complexity
	 * 
	 * @return if there is job ready for executing
	 */
	private boolean jobReady() {
		return schedule.first().isReady();
	}

	/**
	 * Start with executing the first job in the schedule Note: that the first job in the schedule need to be ready TODO fix that the job can be performed earlier that scheduled.
	 * 
	 * @param start
	 *            time of the job
	 */
	protected void executeJob(double time) {
		state = EquipletState.BUSY;
		executing = schedule.pollFirst();

		double latency = time - executing.getStartTime();
		scheduleLatency.put(time, latency);

		executing.updateStartTime(time);
		System.out.printf("EA:%s starts at %.2f (%.2f from scheduled time) with executing job: %s\n", getLocalName(), time, latency, executing);

		informProductProcessing(executing.getProductAgent());

		execute(executing);
	}

	private void execute(Job job) {

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
	@Deprecated
	private void notifyProductArrived(double time, String product, String service) {
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

		if (state == EquipletState.IDLE && jobReady()) {
			historyUpdate(time);
			executeJob(time);
		} else if (state == EquipletState.ERROR && jobReady()) {
			System.out.printf("EA:%s product %s going to be executed after repair\n", getLocalName(), product);
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", getLocalName(), product);
		}
	}

	protected void notifyProductArrived(AID product, double time) {
		// check if it is needed to also check the criteria
		// TODO possible service not necessary, if making the constraint that a
		// product can only have one job ready by an equiplet
		for (Job job : schedule) {
			if (job.getProductAgent().equals(product)) {
				job.setReady();
				break;
			}
		}

		// TODO combine the set ready loop above with the possibility to execute
		// a job that is later in the schedule but can already be performed

		// execute the first job in the schedule if the job is ready

		if (state == EquipletState.IDLE && jobReady()) {
			// begin with executing job that arrived
			historyUpdate(time);
			executeJob(time);
		} else if (state == EquipletState.ERROR && jobReady()) {
			// Equiplet is still broken, but as soon as this is repaired it will execute the first job in the schedule
			System.out.printf("EA:%s product %s going to be executed after repair\n", getLocalName(), product.getLocalName());
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", getLocalName(), product);
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
			// the equiplet should have finished the job, but was broken down in the meantime
			// the equiplet has still a remaining time to continue after the equiplet is repaired
			state = EquipletState.ERROR_FINISHED;
			timeRemaining = time - timeBreakdown;
			System.out.printf("EA:%s job %s should finished but delayed by breakdown, should still %.2f be executed after being repaired.\n", getLocalName(), executing, timeRemaining);
		} else if (state == EquipletState.ERROR_REPAIRED) {
			// the equiplet should have finished with the job, but was broken down in the meantime,
			// the equiplet continues with executing the job
			System.out.printf("EA:%s job %s should finished but delayed by breakdown, should still %.2f be executed.\n", getLocalName(), executing, timeRemaining);
			state = EquipletState.BUSY;
		} else if (state == EquipletState.BUSY) {
			// executing of the job is really finished and will continue with the next job if possible
			executing.updateDueTime(time);
			history.add(executing);
			historyUpdate(time);

			System.out.printf("EA:%s finished with job: %s\n", getLocalName(), executing);

			AID finishedProduct = executing.getProductAgent();

			if (!schedule.isEmpty() && jobReady()) {
				executeJob(time);
			} else {
				state = EquipletState.IDLE;
				executing = null;
			}

			// note that the inform processing is done before inform finished
			// this is because the simulation can delete the product agent (if chosen to do so for performance improvement)
			// therefore there is no guarantee that informing the product is a blocking as the acknowledge is send before notifying the simulation
			informProductStepFinished(finishedProduct);
		} else {
			throw new IllegalArgumentException("EQUIPLET: notify job not given in correct state: " + state);
		}
	}

	/**
	 * Notify the equiplet is broken down A constraint is that the equiplet can only be idle or busy when this can happen
	 * 
	 * @param time
	 *            of the breakdown
	 */
	protected void notifyBreakdown(double time) {
		if (state != EquipletState.IDLE || state != EquipletState.BUSY) {
			throw new IllegalArgumentException("EQUIPLET: notify breakdown not given in correct state: " + state);
		}

		historyUpdate(time);
		state = EquipletState.ERROR;
		timeBreakdown = time;
		System.out.printf("EA:%s is broken down at %.2f\n", getLocalName(), time);
	}

	/**
	 * The notify that the equiplet is repaired if the equiplet has finished during the repair, remember the time the equiplet has broken
	 * 
	 * @param time
	 *            of repair
	 */
	protected void notifyRepaired(double time) {
		if (state == EquipletState.IDLE || state == EquipletState.BUSY || state == EquipletState.ERROR_REPAIRED) {
			throw new IllegalArgumentException("EQUIPLET: notify breakdown not given in correct state: " + state);
		}
		historyUpdate(time);

		if (state == EquipletState.ERROR_FINISHED) {
			// the equiplet has already a finished event received, but is now repaired and can continue with the job
			state = EquipletState.BUSY;
			System.out.printf("EA:%s is repaired at %.2f and continue with job %s \n", getLocalName(), time, executing);
		} else if (state == EquipletState.ERROR_READY) {
			// in the time the equiplet was broken there is a product arrived that can be executed
			executeJob(time);
		} else if (isExecuting()) {
			// the equiplet is executing a job and is repaired, but waits until a job finished event is received
			state = EquipletState.ERROR_REPAIRED;
			timeRemaining = time - timeBreakdown;
			System.out.printf("EA:%s is repaired at %.2f and continue with job %s. The equiplet was %.2f broken.\n", getLocalName(), time, executing, timeRemaining);
		} else if (jobReady()) {
			// when the equiplet was in the error state there became a job ready which arrived before the breakdown
			System.out.println("EAUIPLET ERROR? " + schedule);
			executeJob(time);
			System.out.printf("EA:%s is repaired at %.2f and detect that a job has became ready: %s \n", getLocalName(), time, executing);
		} else {
			// the equiplet has nothing to do and goes into IDLE state
			System.out.printf("EA:%s is repaired at %.2f \n", getLocalName(), time);
			state = EquipletState.IDLE;
		}
	}

	private void informProductProcessing(AID product) {
		try {
			// send product agent information about going to process product
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(product);
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_PROCESSING);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_PROCESSING + System.currentTimeMillis());
			message.setContent(Parser.parseConfirmation(true));
			send(message);

			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template);

			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product processing.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product started to be processed.\n", getLocalName(), executing.getProductAgentName());
			System.err.printf("EA:%s %s", getLocalName(), e.getMessage());
		}
	}

	private void informProductStepFinished(AID product) {
		try {
			// send product agent information about going to process product
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(product);
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_FINISHED + System.currentTimeMillis());
			message.setContent(Parser.parseConfirmation(true));
			send(message);

			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template, 10000);

			if (reply == null || !Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product step finished. %s\n", getLocalName(), reply);
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product step is finished.\n", getLocalName(), executing.getProductAgentName());
			System.err.printf("EA:%s %s", getLocalName(), e.getMessage());
		}
	}

	/**
	 * Validate the schedule, this function should TODO check if there are no jobs that should be performed before a time, but has not. Note: this should be return true also if the
	 * job first becomes ready but a job is being performed before he was scheduled for performence improvement.
	 * 
	 * @param time
	 * @return is the schedule validates
	 */
	private boolean validateSchedule(double time) {
		if (executing == null) {
			for (Job job : schedule) {
				if (job.isReady() && job.getStartTime() < time) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * Information for updating the gui Tuple < name of equiplet, position, services, Tuple < state, waiting, scheduled, executed > >
	 * 
	 * @return information
	 */
	public Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>> getUpdateState() {
		List<String> services = new ArrayList<String>();
		for (Capability capability : capabilities) {
			services.add(capability.getService());
		}
		Tuple<String, Integer, Integer, Integer> info = new Tuple<String, Integer, Integer, Integer>(state.toString(), getWaiting(), getScheduled(), getExecuted());
		return new Tuple<String, Position, List<String>, Tuple<String, Integer, Integer, Integer>>(getLocalName(), position, services, info);
	}
}
