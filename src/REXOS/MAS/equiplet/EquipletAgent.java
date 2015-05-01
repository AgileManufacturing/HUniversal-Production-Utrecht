package MAS.equiplet;

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
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import org.json.JSONException;
import org.json.JSONObject;

import util.DTOModuleSettings;
import util.log.LogLevel;
import util.log.Logger;

import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.BlackboardUpdateException;
//import HAL.exceptions.FactoryException;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
import MAS.util.MASConfiguration;
import MAS.util.Ontology;
import MAS.util.Pair;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;
import MAS.util.Tuple;
import MAS.util.Util;

public class EquipletAgent extends Agent implements HardwareAbstractionLayerListener {

	private static final long serialVersionUID = 1L;
	// private static final double SAFETY_FACTOR = 1;

	// Equiplet knowledge
	protected Position position;
	protected List<Capability> capabilities;

	// Equiplet state
	protected TreeSet<Job> schedule;
	protected EquipletState state;
	protected boolean reconfiguring;
	protected Job executing;
	protected TreeSet<Job> history;
	protected Map<String, Tick> productionTimes;
	
	// Equiplet machine state
	protected String machineState;
	
	// Equiplet
	private HardwareAbstractionLayer hal;
	
	// agent subscriber list
	protected Set<AID> agentListener;

	/**
	 * Equiplet agent startup
	 */
	@Override
	public void setup() {
		Object[] args = getArguments();
		
		if (args != null && args.length > 0) {
			if (args[0].equals("hal")) {
				try {
					hal = new HardwareAbstractionLayer(this);
					System.out.println("EA:" + getLocalName() + " has created HAL");

					ArrayList<String> services = hal.getSupportedServices();

					// these can't be default, all equiplets are different
					Tick defaultServiceDuration = new Tick(10);
					Position defaultPosition = new Position(0, 0);

					// services has to be translated capabilities
					List<Capability> capabilities = new ArrayList<Capability>();
					for (String service : services) {
						// TODO Tick Duration: How long will it be estimated and by who?????????
						// make better suggestion for the time a product step would take
						capabilities.add(new Capability(service, new HashMap<String, Object>(), defaultServiceDuration));
					}

					init(defaultPosition, capabilities);
					
					// initialize Set list for agents that listens to EA
					agentListener = new HashSet<AID>();
					
				} catch (KnowledgeException | BlackboardUpdateException e) {
					e.printStackTrace();
					System.err.printf("EA:%s failed to create the HAL: %s", getLocalName(), e.getMessage());
					return;
				}
			} else {
				// simulation
				try {
					Pair<Position, List<Capability>> configuration = Parser.parseEquipletConfiguration(args[0].toString());

					init(configuration.first, configuration.second);

				} catch (JSONException e) {
					System.err.printf("EA:%s failed to parse the arguments\n", getLocalName());
					System.err.printf("EA:%s %s", getLocalName(), e.getMessage());

					return;
				}
			}
			System.out.printf("EA:%s initialize [pos=%s, capabilties=%s]\n", getLocalName(), position, capabilities);

			register();

			addBehaviour(new EquipletListenerBehaviour(this));
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
		this.reconfiguring = false;
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
	 * [W.I.P. and untested]
	 * This method is called whenever someone needs acces to an equiplet for reconfiguring. 
	 * The equiplet will be safely shut down and send a signal as soon as it has done so (to be implemented in onEquipletStateChanged() )
	 * @param A list of the modules to be removed during this reconfiguring.
	 * @author Thomas Kok
	 * @author Kevin Bosman
	 */
	public void shutdownAndRemove(ArrayList<ModuleIdentifier> arrayList){
		System.out.printf("EA:%s starting to reconfigure.\n", getLocalName());
		this.reconfiguring = true;
		deregister();
		
		// TODO A form a delay should be implemented here, at least until the schedule is empty.
		while(state != EquipletState.IDLE){
			try {
				this.wait();
			} catch (InterruptedException e) {
				Logger.log("Schedule empty.. Continue reconfig shutdown..");
			}
		}
		
		for(ModuleIdentifier removedModule : arrayList){
			try{
				// TODO Transport information on the deleted module to the GKD (The 'result' JSONObject).
				JSONObject result = hal.deleteModule(removedModule);
				Logger.log("The following module has been removed: ", result.get(getName()));
			}catch(Exception ex){
				Logger.log(LogLevel.ERROR, "An error occured while attempting to remove module: ", removedModule);
			}
		}
		
		System.out.printf("EA: %s was deregistered from DF. Equiplet shutdown request sent.\n", getLocalName());
		hal.reconfigureEquiplet();
	}
	
	/**
	 * [W.I.P. and untested]
	 * This method should be called once the reconfiguration of an equiplet has been completed.
	 * It wants a list of modules that were added in the reconfiguration.
	 * @param An object containing two JSONObjects with drivers for the added modules.
	 * @author Thomas Kok
	 * @author Kevin Bosman
	 */
	public boolean startupAndInsert(List<DTOModuleSettings> toBeAddedModuleSettings){
		boolean isInsertingModulesSuccessful = true;
		List<Boolean> results = new ArrayList<Boolean>();
		
		for(DTOModuleSettings moduleSettings : toBeAddedModuleSettings){
			try{
				results.add(hal.insertModule(moduleSettings.staticSettings, moduleSettings.dynamicSettings));
			}catch(InvalidMastModeException ex){
				isInsertingModulesSuccessful = false;
			}
		}
		if(!isInsertingModulesSuccessful || (results.size() != toBeAddedModuleSettings.size())){
			Logger.log("Not all new modules could be added succesfully.");
		}
		
		capabilities.clear();
		ArrayList<String> services = hal.getSupportedServices();
		for (String service : services) {
			capabilities.add(new Capability(service, new HashMap<String, Object>(), new Tick(10)));
		}
		// TODO Dirty as balls. This is done in the initial init function as well, a better solution for this should be implemented.
		this.init(new Position(0,0), capabilities);
		register();
		return isInsertingModulesSuccessful;
	}
	
	public boolean insertModuleFromDatabase(ModuleIdentifier module){
		//hal.insertModule(jsonStaticSettings, jsonDynamicSettings);
//		JSONObject staticSettings = new JSONObject();
//		JSONObject dynamicSettings = new JSONObject();
//		KnowledgeDBClient database = new KnowledgeDBClient();
		
//		hal.insertModule(JouwClass.serialize, jsonDynamicSettings)
		
		return false;
	}
	
	/**
	 * Return all available modules currently listed in the module factory
	 * 
	 * @return List of modules
	 * 
	 * @author Thomas Kok
	 * @author Kevin Bosman
	 */
	public ArrayList<ModuleIdentifier> getAllModules(){
		return hal.getAllModules();
	}

	public void changeMachineStateEquiplet(String machineState){
		try{
			hal.sendEquipletStateCommand(machineState);
		}catch(Exception ex){
			Logger.log("error while sending state change to blackboard");
		}
	}
	
	/**
	 * Euiplet agent clean-up operations
	 */
	@Override
	protected void takeDown() {
		if (!reconfiguring) {
			deregister();
		}
		System.out.printf("EA:%s terminating\n", getLocalName());
	}

	/**
	 * @return the capabilities of the equiplet
	 */
	protected List<Capability> getCapabilities() {
		return capabilities;
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
	 * Get the number of jobs waiting to be executed i.e. ready for execution A job is ready for executing when the
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
	 * Checks whether a job is ready for execution TODO check not only the first in the schedule but also after if job
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
	 * check whether the equiplet can execute a list of product steps within a time frame this returns the list of
	 * services that can be performed @see {@code isCapable}, with the estimate duration of the service and a list of
	 * possible time frames the time frame is a list of times from which the equiplet is free until the equiplet busy
	 * again where the deadline is the time till when is looked
	 * 
	 * @param time
	 *            of the first possibility to perform the product steps
	 * @param deadline
	 *            of the product steps
	 * @param productSteps
	 *            the list of product steps to be performed
	 * @return
	 */
	public synchronized List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> canExecute(Tick time, Tick deadline,
			List<Triple<Integer, String, JSONObject>> productSteps) {
		// answer :: List of services < index in production path, estimate
		// production time, List of from and until time when possible>
		List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> answer = new ArrayList<>();

		for (Triple<Integer, String, JSONObject> productStep : productSteps) {
			if (isCapable(productStep.second, productStep.third)) {
				int index = productStep.first;
				Tick duration = estimateService(productStep.second);
				List<Pair<Tick, Tick>> available = available(time, duration, deadline);
				answer.add(new Triple<Integer, Tick, List<Pair<Tick, Tick>>>(index, duration, available));
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
	 * the first possible time there is enough room in the schedule to perform a service load = 1 - Sr / Sw
	 * 
	 * @param time
	 *            the first possible time from which to look
	 * @param duration
	 *            an estimate of time the equiplet is checked for availability
	 * @return a list of time it is possible to plan the duration TODO can only plan in end of schedule
	 */
	protected synchronized List<Pair<Tick, Tick>> available(Tick time, Tick duration, Tick deadline) {
		List<Pair<Tick, Tick>> available = new ArrayList<Pair<Tick, Tick>>();

		// when ever behind on schedule, the latency would otherwise keep increasing
		Tick window = schedule.isEmpty() ? deadline : deadline.minus(time.minus(schedule.first().getStartTime()).max(0));
		// System.err.println("fix: "+ deadline + " - (" + time + " - "+ (schedule.isEmpty() ? "null" : schedule.first().getStartTime()) + ") = " + deadline +
		// " - "+(schedule.isEmpty() ? "null" : (time.minus(schedule.first().getStartTime()))) + " = "+ window);

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
						} else {
							available.add(new Pair<Tick, Tick>(start, window));
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
	 * calculate the load of the equiplet from a certain time with a window load = 1 - Sr / Sw
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
			if (job.getStartTime().greaterOrEqualThan(time) && job.getStartTime().lessOrEqualThan(time.add(window))
					|| job.getDue().greaterThan(time) && job.getDue().lessOrEqualThan(time.add(window))) {
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
	 * calculate the load of the history of the equiplet from a certain time with a window
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	protected synchronized double loadHistory(Tick time, Tick window) {
		Tick sum = new Tick(0);

		if (isExecuting()) {
			sum = executing.getDue().min(time.add(window)).minus(executing.getStartTime().max(time));
		}

		Iterator<Job> iterator = history.descendingIterator();
		while (iterator.hasNext()) {
			Job job = iterator.next();
			if (job.getStartTime().greaterOrEqualThan(time) && job.getStartTime().lessOrEqualThan(time.add(window))
					|| job.getDue().greaterThan(time) && job.getDue().lessOrEqualThan(time.add(window))) {
				sum = sum.add(job.getDue().min(time.add(window)).minus(job.getStartTime().max(time)));
			} else if (job.getDue().lessThan(time)) {
				// the jobs are outside the scope of the load window
				break;
			}
		}

		if (!MASConfiguration.KEEP_FULL_EQUIPLET_HISORY) {
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
	 * calculate the load of the history of the equiplet from a certain time with a window
	 * 
	 * @param time
	 *            from which the load needs to be calculated
	 * @param window
	 *            of the load
	 * @return load of the equiplet
	 */
	protected synchronized double loadHistory1(Tick time, Tick window) {
		// Tick sum = new Tick(1); // dirty fix
		double sum = 0.0d;
		double t = time.doubleValue();
		double w = t + window.doubleValue();

		if (isExecuting()) {
			sum += time.doubleValue() - executing.getStartTime().doubleValue();
		}

		Iterator<Job> iterator = history.descendingIterator();
		while (iterator.hasNext()) {
			Job job = iterator.next();
			double start = job.getStartTime().doubleValue();
			double due = job.getDue().doubleValue();

			System.out.println("in " + start + " >= " + t + " && " + start + " <= " + w + " || " + due + " > " + t
					+ " && " + due + " <= " + w);
			if (start >= t && start <= w || due > t && due <= w) {
				// if (job.getStartTime().greaterOrEqualThan(time) &&
				// job.getStartTime().lessOrEqualThan(time.add(window)) ||
				// job.getDue().greaterThan(time)
				// && job.getDue().lessOrEqualThan(time.add(window))) {
				// sum +=
				// job.getDue().min(time.add(window)).minus(job.getStartTime().max(time)).doubleValue();
				// } else if (job.getDue().lessThan(time)) {
				sum += Math.min(due, w) - Math.max(start, t);
				System.out.println("sum " + Math.min(due, w) + " - " + Math.max(start, t) + "= "
						+ (Math.min(due, w) - Math.max(start, t)) + " = " + sum);
			} else if (due < t) {
				// the jobs are outside the scope of the load window
				break;
			}
		}

		// double precision error, dirty fix, can use BigDecimal although
		// performance

		System.out.println("sum " + sum);
		sum = sum * 100000000.0 / 100000000.0;
		System.out.println("sum 1 - " + sum + " / " + window.doubleValue() + " = " + (1 - sum / window.doubleValue()));

		return 1 - sum / window.doubleValue();
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
	protected synchronized boolean schedule(AID product, int index, Tick start, Tick deadline, String service,
			JSONObject criteria) {
		// do not schedule a job when going to be reconfigured
		if (reconfiguring) {
			throw new IllegalArgumentException("not able to schedule job when reconfiguring");
			// return false;
		}

		Tick duration = estimateService(service);
		System.out.printf("EA:%s schedule [product=%s, start=%s, duration=%s, deadline=%s, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start, duration, deadline, service, criteria);

		if (schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))).isEmpty()) {
			Job job = new Job(index, product, service, criteria, start, start.add(duration), deadline);
			return schedule.add(job);
		} else {
			// this shouldn't yet occur (not in the simulation), a equiplet
			// should never give a product the available time which cannot be
			// scheduled
			System.err.println("\n----------\nstart=" + start + ", due=" + start.add(duration) + "\nSCHEDULE:\n"
					+ schedule + "\n\n");
			throw new IllegalArgumentException("overlap schedule ");
		}
	}

	/**
	 * Schedule multiple jobs for one equiplet A request consists of a list of product steps, with a tuple of <
	 * production step index, a Pair of <start time, deadline>, service, and criteria >
	 * 
	 * @param product
	 *            agent
	 * @param requests
	 *            list of product step request of multiple product steps
	 * @return true if schedule succeeded
	 */
	protected synchronized boolean schedule(AID product,
			List<Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>> requests) {
		// do not schedule a job when going to be reconfigured
		if (reconfiguring) {
			throw new IllegalArgumentException("not able to schedule job when reconfiguring");
			// return false;
		}

		List<Job> possible = new ArrayList<Job>();
		for (Tuple<Integer, Pair<Tick, Tick>, String, JSONObject> data : requests) {
			int index = data.first;
			Tick start = data.second.first;
			Tick deadline = data.second.second;
			String service = data.third;
			JSONObject criteria = data.fourth;

			// check if the equiplet still capable
			if (isCapable(service, criteria)) {
				// if the request start is after the due time of the executing
				// job
				if (!isExecuting() || start.greaterOrEqualThan(executing.getDue())) {
					Tick duration = estimateService(service);

					System.out.printf("EA:%s schedule [product=%s, index=%d, start=%s, duration=%s, deadline=%s, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), index, start, duration, deadline, service, criteria);

					if (schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))).isEmpty()) {
						Job job = new Job(index, product, service, criteria, start, start.add(duration), deadline);

						possible.add(job);
					} else {
						// this shouldn't yet occur (not in the simulation), a
						// equiplet should never give a product the available
						// time which cannot be scheduled
						System.err.println("\n----------\nstart=" + start + ", due=" + start.add(duration)
								+ "\nSCHEDULE:\n" + schedule + "\n\n");
						throw new IllegalArgumentException("overlap schedule: "
								+ schedule.subSet(new Job(start, start), new Job(start.add(duration), start.add(duration))));
					}
				} else {
					// this shouldn't occur, TODO remove exception and log the
					// error and return false, send fail message which must be
					// handled!
					System.err.printf("EA:%s failed to schedule: request to schedule job starting at %s when still executing %s other job.\n", getLocalName(), start, executing);
					throw new IllegalArgumentException("request to schedule job starting at " + start
							+ " when still executing " + executing + " other job");
				}
			} else {
				// this shouldn't occur, TODO remove exception nd log the error
				// and return false, send fail message which must be handled!
				// maybe possible when reconfigured??? probable not.
				System.err.printf("EA:%s failed to schedule: not capable to execute service %s with criteria %s.\n", getLocalName(), service, criteria);
				throw new IllegalArgumentException("not capable to execute service " + service + " with criteria "
						+ criteria);
			}
		}

		/*
		 * TODO check correctness of this before adding it could be possible that job int job; boolean successful =
		 * true; for (job = 0; job < possible.size(); job++) { successful = schedule.add(possible.get(job)); if
		 * (!successful) { break; } }
		 * 
		 * // rollback added jobs, although first checked if it is possible, there is still gone something wrong if
		 * (!successful) { for (int i = job; i >= 0; i--) { schedule.remove(possible.get(i)); } } return successful;
		 */

		for (Job job : possible) {
			schedule.add(job);
		}

		return true;

	}

	protected synchronized void updateSchedule() {
		// After being broken down and repaired the jobs in the schedule can be
		// delayed.
		// The executing job is updated with the new due time
		// The scheduled jobs should have, depending if the jobs are continuous
		// scheduled, a new start time added with the delay
		// Although the jobs doesn't have to be continuous scheduled, the start
		// time depends on the due date of the previous job
		// The new start time is the max of ( due time of previous job, or the
		// original start time)
		if (isExecuting()) {
			Tick dueTime = executing.getDue();
			for (Job job : schedule) {
				if (dueTime.greaterThan(job.getStartTime())) {
					job.updateStartTime(dueTime);
					dueTime = job.getDue();
				} else {
					// no change in start time, so continuing would not change
					// the schedule
					break;
				}
			}
		}
	}

	@Override
	public String toString() {
		return String.format("%s:[state=%s, \tcapabilities=%s, \texecuting=%s, \tscheduled=%d, \twaiting=%d, \thistory=%d]", getLocalName(), state, capabilities, (executing == null ? "null"
				: executing), schedule.size(), getWaiting(), history.size());
	}

	public String toFullString() {
		return String.format("%s:[state=%s, \tcapabilities=%s, \texecuting=%s, \tscheduled=%d, \twaiting=%d, \thistory=%d] \n\thistory=%s \n\tschedule=%s", getLocalName(), state, capabilities, (executing == null ? "null"
				: executing), schedule.size(), getWaiting(), history.size(), Util.formatSet(history), Util.formatSet(schedule));
	}

	/**
	 * Start with executing the first job in the schedule Note: that the first job in the schedule need to be ready TODO
	 * fix that the job can be performed earlier that scheduled.
	 * 
	 * @param start
	 *            time of the job
	 * @param job
	 *            to be executed
	 */
	protected synchronized void executeJob(Tick time, Job job) {
		Tick latency = time.minus(job.getStartTime());
		state = EquipletState.BUSY;
		executing = job;
		schedule.remove(job);

		executing.updateStartTime(time);
		System.out.printf("EA:%s starts at %s (%s from scheduled time) with executing job: %s\n", getLocalName(), time, latency, executing);

		informProductProcessing(executing.getProductAgent(), time, executing.getIndex());
		execute(executing);
	}

	/**
	 * The actual start of executing a job
	 * 
	 * @param job
	 */
	protected void execute(Job job) {
		System.out.printf("EA:%s executing job: %s\n", getLocalName(), job);
		hal.translateProductStep(job.getService(), job.getCriteria());

	}

	/**
	 * Notify a product is arrived by the equiplet and is ready to be let the equiplet execute his product step
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

		// check if the job arrived is the first in the schedule, than just
		// execute.
		// otherwise, if the first job is too late the arrived job is second,
		// this results in that the already too late (first) job is
		// if there are jobs between the first job and arrived job, swapping of
		// jobs can take place if all deadlines are met.

		//
		// Job ready = null;
		// int index = 0;
		// for (Job job : schedule) {
		// if (job.getProductAgent().equals(product)) {
		// job.setReady();
		//
		// // job can only be executed earlier than planned if equiplet is idle
		// if (state == EquipletState.IDLE) {
		// if (index == 0) {
		// ready = schedule.pollFirst();
		// } else if (index == 1) {
		// // swapping jobs
		// ready = job;
		// schedule.remove(job);
		// Job first = schedule.first();
		//
		// System.out.printf("EA:%s swapping job %s that is ready with %s\n",
		// getLocalName(), ready, first);
		// System.out.printf("EA:%s equiplet=%s\n", getLocalName(), this);
		//
		// // first.updateStartTime(time.add(job.getDuration()));
		// first.updateStartTime(ready.getStartTime());
		//
		// System.out.printf("EA:%s equiplet=%s\n", getLocalName(), this);
		// }
		// }
		//
		// break;
		// }
		// index++;
		// }

		Job arrived = null;
		for (Job job : schedule) {
			if (job.getProductAgent().equals(product)) {
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
			System.out.printf("EA:%s product %s going to be executed after repair\n", getLocalName(), product.getLocalName());
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", getLocalName(), product);
		}
	}

	/**
	 * remove all the job for a product in the schedule
	 * 
	 * @param product
	 *            agent
	 * @return successfulness of removing jobs
	 */
	public synchronized boolean releaseTimeSlopts(AID product) {
		Collection<Job> jobs = new HashSet<>();
		for (Job job : schedule) {
			if (job.getProductAgent().equals(product)) {
				jobs.add(job);
			}
		}
		return schedule.removeAll(jobs);
	}

	/**
	 * Inform the product agent that the job started to be executed is for the product agent Thereafter wait for
	 * confirmation of the message is received
	 * 
	 * @param product
	 *            agent address
	 */
	protected void informProductProcessing(AID product, Tick time, int index) {
		// send product agent information about going to process product
		ACLMessage message = new ACLMessage(ACLMessage.INFORM);
		try {
			message.setContent(Parser.parseProductProcessing(time, index));
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product started to be processed with information [product=%s, time=%s, index=%d].\n", getLocalName(), executing.getProductAgentName(), product, time, index);
			System.err.printf("EA:%s %s\n", getLocalName(), e.getMessage());
			return;
		}

		message.addReceiver(product);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_PRODUCT_PROCESSING);
		message.setReplyWith(Ontology.CONVERSATION_PRODUCT_PROCESSING + System.currentTimeMillis());
		send(message);

		System.out.printf("EA:%s send message to inform product step processing: %s\n", getLocalName(), message.getContent());

		MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
		ACLMessage reply = blockingReceive(template, MASConfiguration.COMMUNICATION_TIMEOUT);

		try {
			if (reply == null || !Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product processing.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product started to be processed with information [product=%s, time=%s, index=%d].\n", getLocalName(), executing.getProductAgentName(), product, time, index);
			System.err.printf("EA:%s %s\n", getLocalName(), e.getMessage());
			System.err.printf("EA:%s reply received: %s\n", getLocalName(), reply);
			throw new IllegalArgumentException("FUCK");
		}
	}

	/**
	 * Inform the product agent the job executed form him is finished
	 * 
	 * @param product
	 *            agents address
	 */
	protected void informProductStepFinished(AID product, Tick time, int index) {
		// send product agent information about going to process product
		ACLMessage message = new ACLMessage(ACLMessage.INFORM);
		message.addReceiver(product);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED);
		message.setReplyWith(Ontology.CONVERSATION_PRODUCT_FINISHED + System.currentTimeMillis());
		try {
			message.setContent(Parser.parseProductFinished(time, index));

		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product step is finished.\n", getLocalName(), (executing != null ? executing.getProductAgentName()
					: "null"));
			System.err.printf("EA:%s %s\n", getLocalName(), e.getMessage());
			return;
		}

		send(message);

		MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
		ACLMessage reply = blockingReceive(template, MASConfiguration.COMMUNICATION_TIMEOUT);

		try {
			if (reply == null || !Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product %s his product step finished. %s\n", getLocalName(), product, reply);
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product step is finished.\n", getLocalName(), (executing != null ? executing.getProductAgentName()
					: "null"));
			System.err.printf("EA:%s %s\n", getLocalName(), e.getMessage());
			System.err.printf("EA:%s reply received: %s\n", getLocalName(), reply);
			throw new IllegalArgumentException("FUCK");
		}
	}
	/**
	 * Add an agent that wants listens to EA
	 * 
	 * @param ID agent id
	 * @author Mitchell van Rijkom
	 */
	public boolean addAgentListener(AID ID){
		if(!agentListener.contains(ID)){
			agentListener.add(ID);
			printRegisteredAgents();
			return true;
		}
		return false;
	}
	
	/**
	 * removes an agent that listens to EA
	 * 
	 * @param ID agent id
	 * @author Mitchell van Rijkom
	 */
	public boolean removeAgentListener(AID ID){
		if(agentListener.contains(ID)){
			agentListener.remove(ID);
			//printRegisteredAgents();
			return true;
		}
		return false;		
	}
	
	/**
	 * This function is meant for debug output to look if the agents are registered to the EA
	 * @author Mitchell van Rijkom 
	 */
	public void printRegisteredAgents(){
		Logger.log("LIST WITH AGENTS/n/n");
		for(AID agents : agentListener){
			Logger.log("AGENT:	" + agents.toString() + "\n");
		}
	}

	@Override
	public void onProcessStatusChanged(HardwareStepStatus status, Module module, HardwareStep hardwareStep) {
		// TODO Auto-generated method stub
		
		// just wrong
		// if(status == HardwareStepStatus.FAILED){
		// equipletActive=false;
		// scheduleCounter=0;
		// equipletSchedule.clear();
		// //Notify Product that failed and remove from schedule.
		// //Log that process execute failed.
		// productStepFailedCounter++;
		// }

	}

	@Override
	public void onExecutionFinished() {
		// executing of the job is finished
		Tick time = new Tick();
		executing.updateDueTime(time);
		history.add(executing);
		System.out.printf("EA:%s finished with job: %s\n", getLocalName(), executing);

		informProductStepFinished(executing.getProductAgent(), time, executing.getIndex());

		// execute the next job
		executing = null;

		Job ready = jobReady();
		if (ready != null) {
			schedule.remove(ready);
			executeJob(time, ready);
		} else {
			state = EquipletState.IDLE;
			this.notifyAll();
		}
	}

	@Override
	public void onExecutionFailed() {
		// TODO Auto-generated method stub

	}

	@Override
	public void onEquipletMachineStateChanged(String state) {
		System.out.println("Machinestate changed: " + state + ".\n");
		// TODO Write actual implementation			
		
		for(AID agent : agentListener){			
			// send can execute reply
			ACLMessage message = new ACLMessage( ACLMessage.INFORM );
			message.addReceiver(agent);
			message.setContent(state);
			send(message);	
			
//			if(this.reconfiguring == true && state == "OFFLINE"){
//			}else if(this.reconfiguring == true && state == "SAFE"){
//			}else if(this.reconfiguring == true && state == "STANDBY"){
//			}else if(this.reconfiguring == true && state == "NORMAL"){
		}		
	}

	@Override
	public void onEquipletModeChanged(String mode) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		System.out.println("module changed: " + state + ".\n");

		for(AID agent : agentListener){			
			// send can execute reply
			ACLMessage message = new ACLMessage( ACLMessage.PROPOSE );
			message.addReceiver(agent);
			message.setContent(module.toString()+ "  " + state);
			send(message);		
			
//			if(this.reconfiguring == true && state == "OFFLINE"){
//			}else if(this.reconfiguring == true && state == "SAFE"){
//			}else if(this.reconfiguring == true && state == "STANDBY"){
//			}else if(this.reconfiguring == true && state == "NORMAL"){
		}
	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onTranslationFinished(String service, JSONObject criteria, ArrayList<HardwareStep> hardwareSteps) {
		System.out.println("EA:" + getLocalName() + " Translating finished, Hardwarestep created, size="
				+ hardwareSteps.size());
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onTranslationFailed(String service, JSONObject criteria) {
		System.err.println("EA:" + getLocalName() + " Translation of productstep has failed: " + service
				+ " with criteria " + criteria);
	}

	@Override
	public String getEquipletName() {
		return this.getLocalName();
	}

	@Override
	public void onReloadEquiplet(String state) {
		// TODO Auto-generated method stub
		
	}
}
