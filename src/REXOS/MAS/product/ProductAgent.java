package MAS.product;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;

import org.json.JSONException;
import org.json.JSONObject;

import MAS.agents.data_classes.MessageType;
import MAS.util.MasConfiguration;
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

	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Triple<LinkedList<ProductStep>, Position, Tick> configuration = Parser
						.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second,
						configuration.third);
				this.created = new Tick();
				this.history = new ArrayList<>();
				this.reschedule = false;

				addBehaviour(new ScheduleBehaviour(this, productSteps));
				addBehaviour(new ProductListenerBehaviour(this));

			} catch (JSONException e) {
				System.err.printf("PA:%s failed to parse the arguments\n",
						getLocalName());
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
			}
		} else {
			System.err.printf("PA:%s Failed to receive correct arguments\n",
					getLocalName());
			state = ProductState.ERROR;
		}
	}

	protected void setup(LinkedList<ProductStep> productSteps,
			Position startPosition, Tick deadline) {
		this.position = startPosition;
		this.productSteps = productSteps;
		this.productionPath = new LinkedList<>();

		this.deadline = deadline;
		this.state = ProductState.SCHEDULING;

		for (ProductStep productStep : productSteps) {
			// replace the criteria in each productstep by the actual
			// identifiers of crates and objects
			productStep.setCriteria(findIdentifiersAndSubjects(productStep
					.getCriteriaasJSON()));
		}

		System.out
				.printf("PA:%s initialize [created=%s, pos=%s, product steps=%s, deadline=%s]\n",
						getLocalName(), getCreated(), position, productSteps,
						deadline);
	}

	/**
	 * This method makes a call to the SupplyAgent. It sends the criteria to it.
	 * In turn the SupplyAgent returns the criteria targets and subjects with
	 * actual targets and subjects (crate codes and coordinates).
	 */
	// TODO Only works with the pick and place actions (the supply agent has to
	// be completely rewritten in order to make it compatible)
	private JSONObject findIdentifiersAndSubjects(JSONObject criteria) {
		if (criteria.length() == 0) {
			return criteria;
		} else {
			AID suppliesagent = new AID();
			suppliesagent.setLocalName("SupplyAgent");
			// suppliesagent.addAddresses(ServerConfigurations.GS_ADDRESS);

			ACLMessage outgoingmessage = new ACLMessage(
					MessageType.SUPPLIER_REQUEST);
			outgoingmessage.addReceiver(suppliesagent);
			outgoingmessage.setContent(criteria.toString());
			outgoingmessage.setSender(this.getAID());
			outgoingmessage.setLanguage("meta");
			this.send(outgoingmessage);

			System.out.println(this.getLocalName() + " has sent message to "
					+ suppliesagent.getLocalName());
			ACLMessage incomingmessage = blockingReceive();
			try {
				JSONObject returnvalue = new JSONObject(
						incomingmessage.getContent());
				return returnvalue;
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return criteria;
		}

	}

	protected void reschedule(Tick time, Tick deadline) {
		this.state = ProductState.SCHEDULING;
		this.reschedule = true;
		LinkedList<ProductStep> steps = new LinkedList<>();
		for (ProductionStep step : productionPath) {
			steps.add(step.getProductStep());
		}
		// System.out.printf("PA:%s start schedule behaviour at %s.\n",
		// getLocalName(), time);
		// scheduleBehaviour.reschedule(steps, time, deadline);
		//
		// System.out.println("schedule "
		// +scheduleBehaviour.getExecutionState());
		// System.out.println("schedule "
		// +listenerBehaviour.getExecutionState());
		//
		// scheduleBehaviour.restart();
		// System.out.println("schedule "
		// +scheduleBehaviour.getExecutionState());
		//
		scheduleBehaviour = new ScheduleBehaviour(this, steps);
		scheduleBehaviour.reschedule(steps, time, deadline);
		addBehaviour(scheduleBehaviour);
		//
		// // wake up behaviours ... the bad way I think
		// ACLMessage msg = new ACLMessage(ACLMessage.UNKNOWN);
		// msg.addReceiver(getAID());
		// // send(msg);
		// System.out.println("schedule "
		// +scheduleBehaviour.getExecutionState());

		// final ScheduleBehaviour scheduleBehaviour = new
		// ScheduleBehaviour(this, steps, time, deadline);

		// addBehaviour(scheduleBehaviour);
		// TODO !!!!!!

		// fock this shit, this is not how it should work
		// scheduleBehaviour.action();
	}

	protected Tick getCreated() {
		return created;
	}

	protected Tick getDeadline() {
		return deadline;
	}

	protected ProductState getProductState() {
		return state;
	}

	protected Position getPosition() {
		return position;
	}

	protected ProductionStep getCurrentStep() {
		return productionPath.peek();
	}

	protected void schedulingFinished(boolean succeeded,
			LinkedList<ProductionStep> path) {
		productionPath = path;
		schedulingFinished(new Tick(), succeeded);
	}

	protected void schedulingFinished(Tick time, boolean succeeded) {
		if (!succeeded) {
			state = ProductState.ERROR;
		}
		// TODO Currently there is no travel time for a product.
		// Once a product is done scheduling it immediately arrives at an
		// equiplet.
		else {
			travelToNextEquiplet();
		}
	}

	@Override
	public String toString() {
		return String
				.format("Product: %s [state=%s, created=%s, position=%s, current step=%s, product steps=%s, path=%s]",
						getLocalName(), state, getCreated(), position,
						(productionPath.size() > 0 ? productionPath.peek()
								: "ERROR"), Arrays.toString(productSteps
								.toArray()), Arrays.toString(productSteps
								.toArray()));
	}

	/**
	 * A for example a travel agent notifies the product agent that he is
	 * arrived by the equiplet
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
			message.setContent(Parser.parseProductArrived(time,	productionStep.getIndex()));
			send(message);
			System.out.println("PA:" + this.getLocalName()	+ " waiting on confirm onproductArrived reply from "
					+ productionStep.getEquiplet().getLocalName()+ " for indexstep " + productionStep.getIndex());

			// receive reply
			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()),MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template , MasConfiguration.COMMUNICATION_TIMEOUT);
			System.out.println("PA:" + this.getLocalName()+ " Received confirm on onproductArrived from "+ reply.getSender().getLocalName() + " for indexstep "
					+ productionStep.getIndex());
			if (!Parser.parseConfirmation(reply.getContent())) {
				System.err.printf("PA:%s failed to receive confirmation.\n",
						getLocalName());
			}
		} catch (JSONException e) {
			System.err
					.printf("PA:%s failed to construct message to equiplet %s for informing product arrived.\n",
							getLocalName(), productionPath.peek()
									.getEquipletName());
			System.err.printf("PA:%s %s ", getLocalName(), e.getMessage());
		}
	}
	
	/*
	 * Is called when the equiplet has finished executing a product step
	 */

	protected void onProductStepFinished(Tick time) {
		// remove the first production step as this is finished
		ProductionStep step = productionPath.pop();
		step.setFinished(time);
		history.add(step);

		if (productionPath.isEmpty()) {
			state = ProductState.FINISHED;
			System.out.println(this.getLocalName() + " is done with all steps");
		} else {
			state = ProductState.TRAVELING;
			travelToNextEquiplet();
		}
	}

	/*
	 * This method is written to solve a problem when using this class during
	 * the simulation. During a "real" run the product can immediatly arrive
	 * with a step at the next equiplet (since there are currently no products
	 * that can require more then one equiplet). This method is overwritten in
	 * the simulation class
	 * TODO has to be rewritten when actual travelling is a thing
	 */
	protected void travelToNextEquiplet() {
		onProductArrived(new Tick());
	}

	protected void onProductProcessing(Tick time) {
		state = ProductState.PROCESSING;
	}

	protected void onProductDelayed(Tick start) {
		Tick delay;
		for (ProductionStep productionStep : productionPath) {

		}
	}

	private void release(Tick time) {
		// release all the time slots planned by equiplets
		HashSet<AID> equiplets = new HashSet<>();
		for (ProductionStep step : productionPath) {
			equiplets.add(step.getEquiplet());
		}

		String replyConversation = Ontology.CONVERSATION_PRODUCT_RELEASE
				+ System.currentTimeMillis();
		MessageTemplate template = MessageTemplate
				.MatchInReplyTo(replyConversation);

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
				System.err
						.printf("PA:%s failed to construct message to equiplet %s for informing product release time slots.\n",
								getLocalName(), equiplet);
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
			}
		}

		// receives the confirmation of the equiplets, whether they released the
		// timeslots
		int received = 0;
		while (received < equiplets.size()) {
			// receive reply
			ACLMessage reply = blockingReceive(template);
			try {
				if (Parser.parseConfirmation(reply.getContent())) {
					received++;
				} else {
					System.err
							.printf("PA:%s failed to receive confirmation for releasing timeslots.\n",
									getLocalName());
				}
			} catch (JSONException e) {
				System.err
						.printf("PA:%s failed to receive message from equiplet %s for releasing timeslots.\n",
								getLocalName(), reply.getSender());
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
			}
		}
	}

	/**
	 * The current product step of product should have been started If this is
	 * not the case reschedule the product steps
	 * 
	 * @param time
	 *            of event
	 * @param index
	 *            of product step
	 */
	protected void onProductStarted(Tick time, int index) {
		Tick newDeadline = deadline.add(deadline.minus(getCreated()));

		System.out
				.printf("PA:%s on product started event [time=%s, index=%d, state=%s, deadline=%s, new deadline=%s, current step=%s, productionPath].\n",
						getLocalName(), time, index, state, deadline,
						newDeadline, getCurrentStep(), productionPath);
		// Check if the equiplet is in the correct state, if Processing
		// everything is correct.
		// When product is waiting check whether the index of current product
		// steps matches
		// The product should never be in other states than waiting and
		// processing
		if (state == ProductState.WAITING
				&& getCurrentStep().getIndex() == index) {
			System.out.printf(
					"PA:%s rescheduling remaining production steps %s.\n",
					getLocalName(), productionPath);

			// release all the time slots planned by equiplets
			release(time);

			// reschedule
			reschedule(time, newDeadline);
		}
	}
}
