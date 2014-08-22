package simulation.mas.equiplet;

import jade.core.AID;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPANames;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.json.JSONException;

import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.Settings;
import simulation.util.Tuple;

public class EquipletAgent extends Equiplet {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

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
	 * Register the equiplet services by the Directory Facilitator Agent
	 */
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
	 * Euiplet agent clean-up operations
	 */
	@Override
	protected void takeDown() {
		System.out.printf("EA:%s terminating\n", getLocalName());
	}

	/**
	 * Schedule a job
	 * 
	 * @param product
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
	protected boolean schedule(AID product, double start, double deadline, String service, Map<String, Object> criteria) {
		double duration = estimateService(service);
		System.out.printf("EA:%s schedule [product=%s, start=%.2f, duration=%.2f, deadline=%.2f, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start, duration, deadline, service, criteria);

		if (schedule.subSet(new Job(start, 0), true, new Job(start, duration), true).isEmpty()) {
			Job job = new Job(product, service, criteria, start, start + duration, deadline);
			return schedule.add(job);
		} else {
			return false;
		}
	}

	/**
	 * Schedule multiple jobs for one product
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
	protected boolean schedule(AID product, List<Tuple<Double, Double, String, Map<String, Object>>> requests) {
		List<Job> possible = new ArrayList<Job>();
		for (Tuple<Double, Double, String, Map<String, Object>> data : requests) {
			double start = data.first;
			double deadline = data.second;
			String service = data.third;
			Map<String, Object> criteria = data.fourth;

			double duration = estimateService(service);

			System.out.printf("EA:%s schedule [product=%s, start=%.2f, duration=%.2f, deadline=%.2f, service=%s, criteria=%s]\n", getLocalName(), product.getLocalName(), start, duration, deadline, service, criteria);

			if (schedule.subSet(new Job(start, 0), true, new Job(start, start + duration), true).isEmpty()) {
				Job job = new Job(product, service, criteria, start, start + duration, deadline);

				possible.add(job);
			} else {

				System.out.println("\n----------\nstart=" + start + ", due=" + (start + duration) + "\nSCHEDULE:\n" + schedule + "\n\n");
				//throw new IllegalArgumentException("Overlap schedule() ");
				// return false;
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
		 * // rollback added jobs, although first checked if it is possible, there is still gone something wrong
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

	@Override
	public String toString() {
		return String.format("%s:[state=%s, capabilities=%s, executing=%s, scheduled=%d, waiting=%d, history=%d, schedule=%s]", getLocalName(), state, capabilities, (state == EquipletState.IDLE ? "null"
				: executing), schedule.size(), getWaiting(), history.size(), schedule);
	}

	/**
	 * Start with executing the first job in the schedule Note: that the first job in the schedule need to be ready
	 * TODO fix that the job can be performed earlier that scheduled.
	 * 
	 * @param start
	 *            time of the job
	 */
	protected void executeJob(double time) {
		state = EquipletState.BUSY;
		executing = schedule.pollFirst();

		executing.updateStartTime(time);
		System.out.printf("EA:%s starts at %.2f with executing job: %s\n", getLocalName(), time, executing);

		informProductProcessing(executing.getProductAgent());

		execute(executing);
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
			executeJob(time);
		} else if (state == EquipletState.ERROR && !isExecuting() && jobReady()) {
			// Equiplet is still broken, but as soon as this is repaired it will execute the first job in the schedule
			System.out.printf("EA:%s product %s going to be executed after repair\n", getLocalName(), product.getLocalName());
			state = EquipletState.ERROR_READY;
		} else {
			System.out.printf("EA:%s product %s is added to waiting products\n", getLocalName(), product);
		}
	}

	/**
	 * Inform the product agent that the job started to be executed is for the product agent
	 * Thereafter wait for confirmation of the message is received
	 * 
	 * @param product
	 *            agent address
	 */
	protected void informProductProcessing(AID product) {
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
			ACLMessage reply = blockingReceive(template, Settings.COMMUNICATION_TIMEOUT);

			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product processing.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product started to be processed.\n", getLocalName(), executing.getProductAgentName());
			System.err.printf("EA:%s %s", getLocalName(), e.getMessage());
		}
	}

	/**
	 * Inform the product agent the job executed form him is finished
	 * 
	 * @param product
	 *            agents address
	 */
	protected void informProductStepFinished(AID product) {
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
			ACLMessage reply = blockingReceive(template, Settings.COMMUNICATION_TIMEOUT);

			if (reply == null || !Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("EA:%s failed to receive confirmation after inform product %s his product step finished. %s\n", getLocalName(), product, reply);
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to construct confirmation message to product %s for informing product step is finished.\n", getLocalName(), executing.getProductAgentName());
			System.err.printf("EA:%s %s", getLocalName(), e.getMessage());
		}
	}
}
