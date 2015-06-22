package MAS.product;

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
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;

import org.json.JSONException;

import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;

public class ProductAgent extends Agent {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private Tick created;
	protected LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	protected ArrayList<ProductionStep> history;

	protected Position position;
	private Tick deadline;
	protected ProductState state;
	protected boolean reschedule;

	private ScheduleBehaviour scheduleBehaviour;
	private ProductListenerBehaviour listenerBehaviour;
	
	private ArrayList<AID> basicListeners;
	private ArrayList<AID> detailedListeners;

	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Triple<LinkedList<ProductStep>, Position, Tick> configuration = Parser.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second, configuration.third);
				this.created = new Tick();
				this.history = new ArrayList<>();
				this.reschedule = false;

				scheduleBehaviour = new ScheduleBehaviour(this, productSteps);
				listenerBehaviour = new ProductListenerBehaviour(this);
				addBehaviour(scheduleBehaviour);
				addBehaviour(listenerBehaviour);

			} catch (JSONException e) {
				System.err.printf("PA:%s failed to parse the arguments\n", getLocalName());
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
			}
		} else {
			System.err.printf("PA:%s Failed to receive correct arguments\n", getLocalName());
			state = ProductState.ERROR;
		}
		
		
		DFAgentDescription dfAgentDescription = new DFAgentDescription();
		dfAgentDescription.setName(getAID());
		//Register as Product agent
		ServiceDescription serviceDescription = new ServiceDescription();
		serviceDescription.setName(this.getLocalName());
		serviceDescription.setType("ProductAgent");
		serviceDescription.addOntologies(Ontology.GRID_ONTOLOGY);
		serviceDescription.addLanguages(FIPANames.ContentLanguage.FIPA_SL);
		dfAgentDescription.addServices(serviceDescription);
		
		try {
			DFService.register(this, dfAgentDescription);
		} catch (FIPAException fe) {
			System.err.printf("EA:%s Failed to register services\n", getLocalName());
			fe.printStackTrace();
		}
		
		listenerBehaviour = new ProductListenerBehaviour(this);
		addBehaviour(listenerBehaviour);
	}

	protected void setup(LinkedList<ProductStep> productSteps, Position startPosition, Tick deadline) {
		this.position = startPosition;
		this.productSteps = productSteps;
		this.productionPath = new LinkedList<>();

		this.deadline = deadline;
		this.state = ProductState.SCHEDULING;

		System.out.printf("PA:%s initialize [created=%s, pos=%s, product steps=%s, deadline=%s]\n", getLocalName(), getCreated(), position, productSteps, deadline);
	}

	private void release(Tick time) {
		// release all the time slots planned by equiplets
		HashSet<AID> equiplets = new HashSet<>();
		for (ProductionStep step : productionPath) {
			equiplets.add(step.getEquiplet());
		}

		String replyConversation = Ontology.CONVERSATION_PRODUCT_RELEASE + System.currentTimeMillis();
		MessageTemplate template = MessageTemplate.MatchInReplyTo(replyConversation);

		for (AID equiplet : equiplets) {
			try {
				ACLMessage message = new ACLMessage(ACLMessage.INFORM);
				message.addReceiver(equiplet);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_PRODUCT_RELEASE);
				message.setReplyWith(replyConversation);
				message.setContent(Parser.parseProductRelease(time));
				send(message);
			} catch (JSONException e) {
				System.err.printf("PA:%s failed to construct message to equiplet %s for informing product release time slots.\n", getLocalName(), equiplet);
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
			}
		}

		// receives the confirmation of the equiplets, whether they released the timeslots
		int received = 0;
		while (received < equiplets.size()) {
			// receive reply
			ACLMessage reply = blockingReceive(template);
			try {
				if (Parser.parseConfirmation(reply.getContent())) {
					received++;
				} else {
					System.err.printf("PA:%s failed to receive confirmation for releasing timeslots.\n", getLocalName());
				}
			} catch (JSONException e) {
				System.err.printf("PA:%s failed to receive message from equiplet %s for releasing timeslots.\n", getLocalName(), reply.getSender());
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
			}
		}
	}

	protected void reschedule(Tick time, Tick deadline) {
		this.state = ProductState.SCHEDULING;
		this.reschedule = true;
		LinkedList<ProductStep> steps = new LinkedList<>();
		for (ProductionStep step : productionPath) {
			steps.add(step.getProductStep());
		}

		Tick newDeadline = time.add(deadline.minus(getCreated()).multiply(2));
		addBehaviour(new ScheduleBehaviour(this, steps, time, newDeadline));
	}

	/**
	 * @return time of agent creation
	 */
	protected Tick getCreated() {
		return created;
	}

	/**
	 * @return deadline of product
	 */
	protected Tick getDeadline() {
		return deadline;
	}

	/**
	 * @return the state of the product
	 */
	protected ProductState getProductState() {
		return state;
	}

	/**
	 * @return current position of agent
	 */
	protected Position getPosition() {
		return position;
	}

	/**
	 * @return the current production step
	 */
	protected ProductionStep getCurrentStep() {
		return productionPath.peek();
	}

	/**
	 * Scheduling behaviour finished and return with a production path
	 * 
	 * @param succeeded
	 * @param path
	 */
	protected void schedulingFinished(Tick time, boolean succeeded, LinkedList<ProductionStep> path) {
		productionPath = path;
		schedulingFinished(time, succeeded);
	}

	/**
	 * the scheduling behaviour finished
	 * 
	 * @param succeeded
	 */
	protected void schedulingFinished(Tick time, boolean succeeded) {
		if (!succeeded) {
			state = ProductState.ERROR;
		} else {
			performNextStep();
		}
	}

	@Override
	public String toString() {
		return String.format("Product: %s [state=%s, \tcreated=%s, \tposition=%s, \tcurrent step=%s, \tproduct steps=%s, \tpath=%s]", getLocalName(), state, getCreated(), position, (productionPath.size() > 0 ? productionPath.peek()
				: "ERROR"), Arrays.toString(productSteps.toArray()), Arrays.toString(productionPath.toArray()));
	}

	/**
	 * A for example a travel agent notifies the product agent that he is arrived by the equiplet
	 */
	protected void onProductArrived(Tick time) {
		// change state from travelling to ready
		state = ProductState.WAITING;

		ProductionStep productionStep = productionPath.peek();
		position = productionStep.getPosition();

		try {
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(productionStep.getEquiplet());
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED);
			message.setReplyWith(Ontology.CONVERSATION_PRODUCT_ARRIVED + System.currentTimeMillis());
			message.setContent(Parser.parseProductArrived(time, productionStep.getIndex()));
			send(message);

			// receive reply
			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template);

			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("PA:%s failed to receive confirmation.\n", getLocalName());
			}
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to construct message to equiplet %s for informing product arrived.\n", getLocalName(), productionPath.peek().getEquipletName());
			System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
		}
	}

	protected void onProductStepFinished(Tick time) {
		// remove the first production step as this is finished
		ProductionStep step = productionPath.pop();
		step.setFinished(time);
		history.add(step);

		if (productionPath.isEmpty()) {
			state = ProductState.FINISHED;
		} else {
			state = ProductState.TRAVELING;
			performNextStep();
		}
	}

	protected void performNextStep() {
		// fix, here need a transport unit or something be informed to travel to next equiplet.
		onProductArrived(new Tick());
		// throw new RuntimeException("perforn next step");
	}

	protected void onProductProcessing(Tick time) {
		state = ProductState.PROCESSING;
	}

	/**
	 * The product is delayed
	 * 
	 * @param start
	 */
	protected void onProductDelayed(Tick start) {

	}

	/**
	 * The current product step of product should have been started
	 * If this is not the case reschedule the product steps
	 * 
	 * @param time
	 *            of event
	 * @param index
	 *            of product step
	 */
	protected void onProductStarted(Tick time, int index) {
		Tick newDeadline = deadline.add(deadline.minus(getCreated()));

		System.out.printf("PA:%s on product started event [time=%s, index=%d, state=%s, deadline=%s, new deadline=%s, current step=%s, productionPath].\n", getLocalName(), time, index, state, deadline, newDeadline, getCurrentStep(), productionPath);
		// Check if the equiplet is in the correct state, if Processing everything is correct.
		// When product is waiting check whether the index of current product steps matches
		// The product should never be in other states than waiting and processing
		if (state == ProductState.WAITING && getCurrentStep().getIndex() == index) {
			System.out.printf("PA:%s rescheduling remaining production steps %s.\n", getLocalName(), productionPath);

			// release all the time slots planned by equiplets
			release(time);

			// reschedule
			reschedule(time, newDeadline);
		}
	}
	
	public LinkedList<ProductStep> getProductSteps() {
		return this.productSteps;
	}
	
	public LinkedList<ProductionStep> getProductionPath() {
		return this.productionPath;
	}
	
	public ArrayList<ProductionStep> getHistory() {
		return this.history;
	}
	
	public void addSCADADetailedListener(AID listener) {
		if(!detailedListeners.contains(listener)) {
			detailedListeners.add(listener);
		}
	}
	
	public void addSCADABasicListener(AID listener) {
		if(!basicListeners.contains(listener)) {
			basicListeners.add(listener);
		}
	}
	
/*	private void sendUpdateMessageToBasicListeners(String update){
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		for(int i = 0; i < basicListeners.size(); i++) {
			message.addReceiver(basicListeners.get(i));
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
			message.setContent(update);
			send(message);
		}
		
	}
	
	private void sendUpdateMessageToDetailedListeners(String update){
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		for(int i = 0; i < basicListeners.size(); i++) {
			message.addReceiver(basicListeners.get(i));
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_LISTENER_COMMAND);
			message.setContent(update);
			send(message);
		}
	}*/
}
