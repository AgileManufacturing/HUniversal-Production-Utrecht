package simulation.mas;

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

import simulation.util.Capability;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.ProductStep;
import simulation.util.Ontology;
import simulation.util.Triple;
import simulation.util.Tuple;

public class EquipletAgent extends Agent {

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
	private double timeBreakdown;
	private double timeRemaining;

	// Statistics containing time in state <IDLE, BUSY, ERROR>
	private Triple<Double, Double, Double> statistics;
	private double lastHistoryUpdate;

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

				this.schedule = new TreeSet<>();
				this.history = new ArrayList<>();

				this.state = EquipletState.IDLE;
				this.executing = null;

				lastHistoryUpdate = 0;
				statistics = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);

				this.timeBreakdown = -1;
				this.timeRemaining = -1;

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
		System.out.printf("EQ:%s terminating\n", getLocalName());
	}

	/**
	 * Get the state of the equiplet
	 * 
	 * @return the equiplet state
	 */
	protected EquipletState getEquipletState() {
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

	public List<Triple<Integer, Double, List<Pair<Double, Double>>>> canExecute(double time, List<ProductStep> productSteps) {
		// answer :: List of services < index in production path, estimate production time, List of from and until time when possible>
		List<Triple<Integer, Double, List<Pair<Double, Double>>>> answer = new ArrayList<>();

		for (ProductStep productStep : productSteps) {
			if (isCapable(productStep.getService(), productStep.getCriteria())) {
				int index = productStep.getIndex();
				double duration = estimateService(productStep.getService());
				List<Pair<Double, Double>> available = available(time, duration);
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

	protected List<Job> getHistory() {
		return history;
	}

	/**
	 * @return the remaining process time of the job
	 */
	protected double getRemainingTime() {
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
		if (state == EquipletState.IDLE) {
			statistics.first += elapsed;
		} else if (state == EquipletState.BUSY) {
			statistics.second += elapsed;
		} else if (state == EquipletState.ERROR) {
			statistics.third += elapsed;
		}
	}

	/**
	 * Retrieve the statistics of the equiplet
	 * 
	 * @param time
	 *            for last update to include time of question
	 * @return time the equiplet is <IDLE, BUSY, ERROR>
	 */
	protected Triple<Double, Double, Double> getStatistics(double time) {
		historyUpdate(time);
		return statistics;
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
		System.out.printf("EA:%s schedule [product=%s, start=%.2f, duration=%.2f, deadline=%.2f, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start,
				duration, deadline, service, criteria);

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
	protected List<Pair<Double, Double>> available(double time, double duration) {
		// TODO fix this so job can be scheduled in between jobs instead of
		// always behind the last
		List<Pair<Double, Double>> available = new ArrayList<Pair<Double, Double>>();
		if (schedule.size() > 0) {
			Job job = schedule.last();
			available.add(new Pair<Double, Double>(job.getDueTime(), job.getDueTime() + 1000));
			return available;
		} else {
			available.add(new Pair<Double, Double>(time, time + 1000));
			return available;
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
	 * TODO RENAME function name The simulation need to check if the just scheduled product step is going to be executed Simulation check whether there is need for scheduling a job
	 * finished event
	 * 
	 * @param product
	 *            name
	 * @param service
	 * @param map
	 * @return is the job
	 */
	public boolean isExecutingStep(String product, String service, Map<String, Object> map) {

		// System.out.println(" isExecutingStep " + executing + " " + exe)
		return (executing != null && executing.getProductAgentName().equalsIgnoreCase(product) && executing.getService().equalsIgnoreCase(service) && executing.getCriteria()
				.equals(map));
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
	protected String getExecutingProduct() {
		return executing.getProductAgentName();
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
		executing.updateStartTime(time);
		System.out.printf("EQ:%s starts at %.2f with executing job: %s\n", name, time, executing);

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

		if (state == EquipletState.IDLE && schedule.first().isReady()) {
			historyUpdate(time);
			executeJob(time);

			informProductProcessing();
		} else if (state == EquipletState.ERROR && schedule.first().isReady()) {
			System.out.printf("EQ:%s product %s going to be executed after repair\n", name, product);
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EQ:%s product %s is added to waiting products\n", name, product);
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

		if (state == EquipletState.IDLE && schedule.first().isReady()) {
			historyUpdate(time);
			executeJob(time);

			informProductProcessing();
		} else if (state == EquipletState.ERROR && schedule.first().isReady()) {
			System.out.printf("EA:%s product %s going to be executed after repair\n", name, product);
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", name, product);
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
			state = EquipletState.ERROR_FIXED;
			timeRemaining = time - timeBreakdown;
		} else if (state == EquipletState.ERROR_FIXED) {
			System.out.printf("EQ:%s job %s should finished but delayed by breakdown, should still %.2f be executed\n", name, executing, timeRemaining);
			state = EquipletState.BUSY;
		} else {
			executing.updateDueTime(time);
			history.add(executing);

			System.out.printf("EQ:%s finished with job %s\n", name, executing);

			historyUpdate(time);
			if (!schedule.isEmpty() && schedule.first().isReady()) {
				executeJob(time);

				informProductProcessing();
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
	protected void notifyBreakdown(double time) {
		historyUpdate(time);
		state = EquipletState.ERROR;
		timeBreakdown = time;
		System.out.printf("EQ:%s is broken down at %.2f\n", name, time);
	}

	/**
	 * The notify that the equiplet is repaired if the equiplet has finished during the repair, remember the time the equiplet has broken
	 * 
	 * @param time
	 *            of repair
	 */
	protected void notifyRepaired(double time) {
		historyUpdate(time);

		if (state == EquipletState.ERROR_FINISH) {
			state = EquipletState.BUSY;
			System.out.printf("EQ:%s is repaired at %.2f and continue with job %s \n", name, time, executing);
		} else if (state == EquipletState.ERROR_READY) {
			executeJob(time);
		} else if (isExecuting()) {
			state = EquipletState.ERROR_FIXED;
			System.out.printf("EQ:%s is repaired at %.2f and continue with job %s \n", name, time, executing);
		} else {
			System.out.printf("EQ:%s is repaired at %.2f \n", name, time);
			state = EquipletState.IDLE;
		}
	}

	private void informProductProcessing() {
		try {
			// send product agent information about going to process product
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(executing.getProductAgent());
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_PROCESSING);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_PROCESSING + System.currentTimeMillis());
			message.setContent(Parser.parseConfirmation(true));
			send(message);
			
			MessageTemplate template = MessageTemplate
					.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template);
			
			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product processing.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product started to be processed.\n", getLocalName(),
					executing.getProductAgentName());
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
