package MAS.simulation.mas.product;

import jade.core.AID;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.json.JSONException;

import MAS.simulation.simulation.ISimulation;
import MAS.simulation.util.Ontology;
import MAS.simulation.util.Pair;
import MAS.simulation.util.Parser;
import MAS.simulation.util.Position;
import MAS.simulation.util.Settings;
import MAS.simulation.util.Tick;
import MAS.simulation.util.Triple;

public class ProductAgentSimDealWithIt extends ProductAgent implements IProductSim {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ISimulation simulation;

	// TODO created has become a Tick, so maybe it is not needed to store here, but let the real agent handle the tick, or describe way this is here
	private Tick created;
	private int currentStepIndex;
	private ProductionStep currentStep;

	public ProductAgentSimDealWithIt(ISimulation simulation, LinkedList<ProductStep> productSteps, Position position, Tick time, Tick deadline) {
		try {
			Object[] args = new Object[] { Parser.parseProductConfiguration(productSteps, position, deadline) };
			setArguments(args);
		} catch (JSONException e) {
			System.err.printf("PA: failed to create product: %s.\n", e.getMessage());
		}

		this.simulation = simulation;
		this.created = time;
	}

	/**
	 * start of product agent,
	 * override method to prevent equiplet starting schedule behaviour
	 */
	@Override
	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Triple<LinkedList<ProductStep>, Position, Tick> configuration = Parser.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second, configuration.third);
				this.currentStepIndex = 0;
				this.currentStep = null;
				this.history = new ArrayList<ProductionStep>();

				addBehaviour(new ProductListenerBehaviour(this));
				executeNextProductStep();
				simulation.notifyProductCreated(getLocalName(), currentStep.getEquipletName());
			} catch (JSONException e) {
				System.err.printf("PA:%s failed to parse the arguments\n", getLocalName());
				System.err.printf("PA:%s %s\n", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
				simulation.notifyProductCreationFailed(getLocalName());
				simulation.log(Settings.PRODUCT_LOG, getLocalName(), "PA:" + getLocalName() + " failed: " + productSteps);
			} catch (SchedulingException e) {
				System.err.printf("PA:%s failed to schedule product\n", getLocalName());
				System.err.printf("PA:%s %s\n", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
				simulation.notifyProductCreationFailed(getLocalName());
				simulation.log(Settings.PRODUCT_LOG, getLocalName(), "PA:" + getLocalName() + " failed: " + productSteps);
			}
		} else {
			System.err.printf("PA:%s Failed to receive correct arguments\n", getLocalName());
			state = ProductState.ERROR;
			simulation.notifyProductCreationFailed(getLocalName());
			simulation.log(Settings.PRODUCT_LOG, getLocalName(), "PA:" + getLocalName() + " failed: " + productSteps);
		}
	}

	@Override
	public void kill() {
		// TODO handle properly, let the equiplet know that I been killed
		System.out.printf("PA:%s terminating\n", getLocalName());
		doDelete();
	}

	@Override
	public Position getPosition() {
		return super.getPosition();
	}

	@Override
	public Tick getCreated() {
		return created;
	}

	@Override
	public Tick getDeadline() {
		return super.getDeadline();
	}

	@Override
	protected ProductionStep getCurrentStep() {
		return currentStep;
	}

	/**
	 * A for example a travel agent notifies the product agent that he is arrived by the equiplet
	 */
	@Override
	public void onProductArrived(Tick time) {
		// change state from travelling to ready
		state = ProductState.WAITING;

		if (currentStep == null) {
			throw new IllegalArgumentException("traveling to position is not set.");
		}

		currentStep.updateStart(time);
		position = currentStep.getPosition();

		try {
			ArrayList<ProductionStep> steps = new ArrayList<ProductionStep>();
			steps.add(currentStep);
			ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
			message.addReceiver(currentStep.getEquiplet());
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_SCHEDULE);
			message.setReplyWith(Ontology.CONVERSATION_SCHEDULE + System.currentTimeMillis());
			message.setContent(Parser.parseScheduleRequest(steps, getDeadline()));
			send(message);

			// receive reply
			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template);

			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("PA:%s failed to receive confirmation.\n", getLocalName());
			}

			message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(currentStep.getEquiplet());
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_ARRIVED + System.currentTimeMillis());
			message.setContent(Parser.parseProductArrived(time, currentStep.getIndex()));
			send(message);

			// receive reply
			template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			reply = blockingReceive(template);

			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("PA:%s failed to receive confirmation.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to construct message to equiplet %s for informing product arrived.\n", getLocalName(), currentStep.getEquipletName());
			System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
		}
	}

	@Override
	protected void onProductStepFinished(Tick time) {
		// remove the first production step as this is finished
		currentStep.setFinished(time);
		history.add(currentStep);
		currentStep = null;
		currentStepIndex++;

		// After regular behaviour when a product step is finished, inform also the simulation
		if (currentStepIndex >= productSteps.size()) {
			state = ProductState.FINISHED;
			// notify the simulation that the product is finished
			simulation.notifyProductFinished(getLocalName());
			simulation.log(Settings.PRODUCT_LOG, getLocalName(), "PA:" + getLocalName() + " finished: " + history);
		} else {
			try {

				// find next equiplet, goto, and notify the simulation that the product is traveling
				executeNextProductStep();

				simulation.notifyProductTraveling(getLocalName(), currentStep.getEquipletName());
			} catch (SchedulingException e) {
				System.err.printf("PA:%s failed to schedule product\n", getLocalName());
				System.err.printf("PA:%s %s\n", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
				simulation.notifyProductOverdue(getLocalName());

				simulation.log(Settings.PRODUCT_LOG, getLocalName(), "PA:" + getLocalName() + " overdue: " + history + ", product steps " + productSteps);
			}
		}
	}

	@Override
	protected void onProductProcessing(Tick time) {
		super.onProductProcessing(time);

		// notify the simulation that processing begins
		ProductionStep step = getCurrentStep();
		currentStep.updateStart(time);
		simulation.notifyProductProcessing(getLocalName(), step.getEquipletName(), step.getService(), step.getIndex());
	}

	@Override
	protected void schedulingFinished(boolean succeeded) {
		System.out.println("scheduling finished");
		// let the simulation know that the creation of product agent failed
		if (succeeded) {
			simulation.notifyProductCreated(getLocalName(), getCurrentStep().getEquipletName());
		} else {
			simulation.notifyProductCreationFailed(getLocalName());
			simulation.log(Settings.PRODUCT_LOG, getLocalName(), "PA:" + getLocalName() + " failed: " + productSteps);
		}
	}

	private void executeNextProductStep() throws SchedulingException {
		ProductStep productStep = productSteps.get(currentStepIndex);

		// search for equiplets that are capable to perform product step
		List<AID> suitedEquiplets = new ArrayList<AID>();
		try {
			// Build the description used as template for the search
			DFAgentDescription template = new DFAgentDescription();
			ServiceDescription sd = new ServiceDescription();
			sd.setType(Ontology.SERVICE_SEARCH_TYPE);
			sd.setName(productStep.getService());

			template.addServices(sd);

			DFAgentDescription[] results = DFService.search(this, template);
			if (results.length > 0) {
				for (int i = 0; i < results.length; i++) {
					AID equiplet = results[i].getName();
					suitedEquiplets.add(equiplet);
				}
			} else {
				System.err.printf("PA:%s failed to find the service %s\n", getLocalName(), productStep.getService());
			}
		} catch (FIPAException fe) {
			fe.printStackTrace();
			throw new SchedulingException(" failed to find services: " + fe.getMessage());
		}

		if (suitedEquiplets.isEmpty()) {
			throw new SchedulingException(" failed to find services for: " + productStep);
		}

		// question the equiplets if they are capable
		String replyConversation = Ontology.CONVERSATION_CAN_EXECUTE + System.currentTimeMillis();
		for (AID equiplet : suitedEquiplets) {
			try {
				LinkedList<ProductStep> steps = new LinkedList<>();
				steps.add(productStep);

				ACLMessage message = new ACLMessage(ACLMessage.QUERY_REF);
				message.addReceiver(equiplet);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_CAN_EXECUTE);
				message.setReplyWith(replyConversation);
				message.setContent(Parser.parseCanExecute(getCreated(), getDeadline(), steps));
				send(message);
			} catch (JSONException e) {
				System.err.printf("PA:%s failed to construct message to equiplet for asking can execute %s.\n", getLocalName(), productStep);
				System.err.printf("PA:%s %s\n", getLocalName(), e.getMessage());
			}
		}

		System.out.printf("PA:%s waiting on answers of equiplets whether they can execute the product steps...\n", getLocalName());

		MessageTemplate template = MessageTemplate.MatchInReplyTo(replyConversation);

		// receives the answers of the equiplets, whether they can execute the product steps
		// choose an equiplet and goto the equiplet
		AID bestEquiplet = null;
		Position equipletPos = null;
		Tick duration = null;
		double score = 0; // load score, 1 is low and 0 is high load

		int counter = 0;
		while (counter < suitedEquiplets.size()) {
			ACLMessage msg = blockingReceive(template, Settings.COMMUNICATION_TIMEOUT);

			// the equiplet is able to perform the product step and propose possibilities for executing the product step
			if (msg != null && msg.getPerformative() == ACLMessage.PROPOSE) {
				counter++;

				System.out.printf("PA:%s can execute reply received from %s : %s.\n", getLocalName(), msg.getSender().getLocalName(), msg.getContent());
				try {
					// Triple < List of product steps, load, position, options >
					Triple<List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>>, Double, Position> answer = Parser.parseCanExecuteAnswer(msg.getContent());
					double load = answer.second;
					if (load > score) {
						score = load;
						bestEquiplet = msg.getSender();
						equipletPos = answer.third;
						duration = answer.first.size() > 0 ? answer.first.get(0).second : new Tick(0);
					}
				} catch (JSONException e) {
					System.err.printf("PA:%s failed to receive correct message from equiplet %s when asking can execute %s.\n", getLocalName(), msg.getSender().getLocalName(), msg.getContent());
					System.err.printf("PA:%s %s\n", getLocalName(), e.getMessage());
				}
			} else if (msg != null && msg.getPerformative() == ACLMessage.DISCONFIRM) {
				// equiplet is not able to execute the product step
			} else if (msg == null) {
				// TODO remove!
				throw new SchedulingException("message not received while waiting on equiplet reply if equiplet can execute product step");
			}
		}

		// goto equiplet
		if (bestEquiplet != null) {
			state = ProductState.TRAVELING;
			currentStep = new ProductionStep(productStep, bestEquiplet, equipletPos, created, duration);
		} else {
			throw new SchedulingException("failed to find capable equiplet with enough room, while number of suited equiplets " + suitedEquiplets.size() + " ");
		}
	}
}
