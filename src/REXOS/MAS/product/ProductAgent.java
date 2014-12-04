package MAS.product;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.Arrays;
import java.util.LinkedList;

import org.json.JSONException;
import org.json.JSONObject;

import MAS.agents.data_classes.MessageType;
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
	private LinkedList<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	private Position position;
	private Tick deadline;
	private ProductState state;

	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Triple<LinkedList<ProductStep>, Position, Tick> configuration = Parser.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second, configuration.third);
				this.created = new Tick();

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

	protected void setup(LinkedList<ProductStep> productSteps, Position startPosition, Tick deadline) {
		this.position = startPosition;
		this.productSteps = productSteps;
		this.productionPath = new LinkedList<>();
		
		this.deadline = deadline;
		this.state = ProductState.SCHEDULING;	
		
		for (ProductStep productStep : productSteps) {
			//replace the criteria in each productstep by the actual identifiers of crates and objects
			productStep.setCriteria(findIdentifiersAndSubjects(productStep.getCriteriaasJSON())); 
		}
		
		System.out.printf("PA:%s initialize [created=%s, pos=%s, product steps=%s, deadline=%s]\n", getLocalName(), getCreated(), position, productSteps, deadline);
	}
	
	
	/**
	 * This method makes a call to the SupplyAgent. It sends the criteria to it. In turn the SupplyAgent returns the
	 * criteria targets and subjects with actual targets and subjects (crate codes and coordinates). 
	 */
	//TODO Only works with the pick and place actions (the supply agent has to be completely rewritten in order to make it compatible)
	private JSONObject findIdentifiersAndSubjects(JSONObject criteria) {
		if(criteria.length() == 0) {
			return criteria;
		}
		else {
			AID suppliesagent = new AID();
			suppliesagent.setLocalName("SupplyAgent");
			//suppliesagent.addAddresses(ServerConfigurations.GS_ADDRESS);
			
			ACLMessage outgoingmessage = new ACLMessage(MessageType.SUPPLIER_REQUEST);
			outgoingmessage.addReceiver(suppliesagent);
			outgoingmessage.setContent(criteria.toString());
			outgoingmessage.setSender(this.getAID());
			outgoingmessage.setLanguage("meta");
			this.send(outgoingmessage); 
			
			System.out.println(this.getLocalName() + " has sent message to " + suppliesagent.getLocalName());
			ACLMessage incomingmessage = blockingReceive();
			try {
				JSONObject returnvalue = new JSONObject(incomingmessage.getContent());
				return returnvalue;
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return criteria;
		}
		
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

	protected void schedulingFinished(boolean succeeded, LinkedList<ProductionStep> path) {
		productionPath = path;
		schedulingFinished(succeeded);
	}

	protected void schedulingFinished(boolean succeeded) {
		if (!succeeded) {
			state = ProductState.ERROR;
		}
		//TODO Currently there is no travel time for a product.
		//Once a product is done scheduling it immediately arrives at an equiplet.
		else{
			onProductArrived(new Tick(1));
		}
	}

	@Override
	public String toString() {
		return String.format("Product: %s [state=%s, created=%s, position=%s, current step=%s, product steps=%s, path=%s]", getLocalName(), state, getCreated(), position, (productionPath.size() > 0 ? productionPath.peek()
				: "ERROR"), Arrays.toString(productSteps.toArray()), Arrays.toString(productSteps.toArray()));
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
			System.out.println("PA: waiting on onproductArrived reply");
			
			// receive reply
			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchConversationId(message.getConversationId()), MessageTemplate.MatchInReplyTo(message.getReplyWith()));
			ACLMessage reply = blockingReceive(template, 1000);
			System.out.println("PA:" + this.getLocalName() +  " Received confirm on onproductArrived");
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
			System.out.println(this.getLocalName() + " is done with all steps");
		} else {
			state = ProductState.TRAVELING;
			onProductArrived(new Tick(1));
		}
	}


	protected void onProductProcessing() {
		state = ProductState.PROCESSING;
	}

	protected void onProductDelayed(Tick start) {
		Tick delay;
		for (ProductionStep productionStep : productionPath) {

		}
	}
}
