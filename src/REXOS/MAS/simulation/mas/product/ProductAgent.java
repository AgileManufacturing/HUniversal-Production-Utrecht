package simulation.mas.product;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.Arrays;
import java.util.LinkedList;

import org.json.JSONException;

import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;

public class ProductAgent extends Agent {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private double created;
	private LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	private Position position;
	private double deadline;
	private ProductState state;

	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Pair<LinkedList<ProductStep>, Position> configuration = Parser.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second);
				this.created = Double.valueOf(System.currentTimeMillis());

				addBehaviour(new ScheduleBehaviour(this, productSteps));
				addBehaviour(new ProductListenerBehaviour(this));

			} catch (JSONException e) {
				System.err.printf("PA:%s failed to parse the arguments\n", getLocalName());
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
			}
		} else {
			System.err.printf("PA:%s Failed to receive correct arguments\n", getLocalName());
			state = ProductState.ERROR;
		}
	}

	protected void setup(LinkedList<ProductStep> productSteps, Position startPosition) {
		this.position = startPosition;
		this.productSteps = productSteps;
		this.productionPath = new LinkedList<>();

		this.deadline = getCreated() + 10000;
		this.state = ProductState.SCHEDULING;
		
		System.out.printf("PA:%s initialize [created=%.2f, pos=%s, product steps=%s, deadline=%.0f]\n", getLocalName(), getCreated(), position, productSteps, deadline);
	}

	protected double getCreated() {
		return created;
	}

	protected double getDeadline() {
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

	protected void schedulingFinished(boolean succeeded, LinkedList<ProductionStep> path) {
		productionPath = path;
		schedulingFinished(succeeded);
	}

	protected void schedulingFinished(boolean succeeded) {
		if (!succeeded) {
			state = ProductState.ERROR;
		}
	}

	@Override
	public String toString() {
		return String.format("Product: %s [state=%s, created=%.2f, position=%s, current step=%s, product steps=%s, path=%s]", getLocalName(), state, getCreated(), position, (productionPath.size() > 0 ? productionPath.peek()
				: "ERROR"), Arrays.toString(productSteps.toArray()), Arrays.toString(productSteps.toArray()));
	}

	/**
	 * A for example a travel agent notifies the product agent that he is arrived by the equiplet
	 */
	protected void onProductArrived(double time) {
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
			message.setContent(Parser.parseProductArrived(time));
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

	protected void onProductStepFinished() {
		// remove the first production step as this is finished
		productionPath.pop();

		if (productionPath.isEmpty()) {
			state = ProductState.FINISHED;
		} else {
			state = ProductState.TRAVELING;
		}
	}

	protected void onProductProcessing() {
		state = ProductState.PROCESSING;
	}
	
	protected void onProductDelayed(double start) {
		double delay;
		for (ProductionStep productionStep : productionPath) {

		}
	}
}
