package simulation.mas.product;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.json.JSONException;

import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.Triple;

public class ProductAgent extends Agent {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private double created;
	private List<ProductStep> productSteps;
	private LinkedList<ProductionStep> productionPath;
	private Position position;
	private double deadline;
	private ProductState state;

	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Pair<List<ProductStep>, Position> configuration = Parser.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second);
				this.created = Double.valueOf(System.currentTimeMillis());

				addBehaviour(new ScheduleBehaviour(this, productSteps));
				// /addBehaviour(new ScheduleBehaviours());
				addBehaviour(new ProductListenerBehaviour());

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

	public void setup(List<ProductStep> productSteps, Position startPosition) {
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

	public class ProductListenerBehaviour extends Behaviour {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			MessageTemplate template = MessageTemplate.not(MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.DISCONFIRM), MessageTemplate.MatchPerformative(ACLMessage.CONFIRM)));
			ACLMessage msg = blockingReceive(template);
			if (msg != null) {
				System.out.printf("PA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());
				switch (msg.getPerformative()) {
				case ACLMessage.INFORM:
					if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_PROCESSING)) {
						try {
							boolean confirmation = Parser.parseConfirmation(msg.getContent());
							if (confirmation) {
								onProductProcessing();
							} else {
								System.err.printf("PA:%s failed to receive confirmation.\n", getLocalName());
							}

							ACLMessage reply = msg.createReply();
							reply.setContent(Parser.parseConfirmation(confirmation));
							send(reply);
						} catch (JSONException e) {
							System.err.printf("PA:%s failed to parse confirmation\n", getLocalName());
							System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
						}
					}

					if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_FINISHED)) {
						try {
							boolean confirmation = Parser.parseConfirmation(msg.getContent());
							if (confirmation) {
								onProductStepFinished();
							} else {
								System.err.printf("PA:%s failed to receive confirmation.\n", getLocalName());
							}

							ACLMessage reply = msg.createReply();
							reply.setContent(Parser.parseConfirmation(confirmation));
							send(reply);
						} catch (JSONException e) {
							System.err.printf("PA:%s failed to parse confirmation\n", getLocalName());
							System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
						}
					}
					break;
				default:
					break;
				}
			}
		}

		@Override
		public boolean done() {
			return false;
		}
	}

	@Deprecated
	public class ScheduleBehaviours extends Behaviour {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;
		private boolean searched;
		private boolean scheduled;
		private int retry = 1;

		/**
		 * @var equipletInfo equiplet info :: Map < equiplet name, Pair < load of equiplet, position of equiplet> >
		 */
		private Map<AID, Pair<Double, Position>> equipletInfo;

		public ScheduleBehaviours() {
			searched = false;
			scheduled = false;

			equipletInfo = new HashMap<AID, Pair<Double, Position>>();

			state = ProductState.SCHEDULING;
		}

		@Override
		public void action() {
			System.out.printf("PA:%s starts schedule behaviour, product steps: %s\n", getLocalName(), productSteps);

			HashMap<AID, LinkedList<ProductStep>> suitedEquiplets = new HashMap<AID, LinkedList<ProductStep>>();
			if (!searched) {
				suitedEquiplets = searchSuitedEquiplets();
				searched = !suitedEquiplets.isEmpty();
			}

			if (searched && !scheduled) {
				System.out.printf("PA:%s find the following suited equiplets %s\n", getLocalName(), suitedEquiplets);

				// send a questions to each of the equilplet if the product step can be executed
				for (Entry<AID, LinkedList<ProductStep>> entry : suitedEquiplets.entrySet()) {
					try {
						ACLMessage message = new ACLMessage(ACLMessage.QUERY_REF);
						message.addReceiver(entry.getKey());
						message.setOntology(Ontology.GRID_ONTOLOGY);
						message.setConversationId(Ontology.CONVERSATION_CAN_EXECUTE);
						message.setReplyWith(Ontology.CONVERSATION_CAN_EXECUTE + System.currentTimeMillis());
						message.setContent(Parser.parseCanExecute(getCreated(), deadline, entry.getValue()));
						send(message);
					} catch (JSONException e) {
						System.err.printf("PA:%s failed to construct message to equiplet for asking can execute %s.\n", getLocalName(), entry.getValue());
						System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
					}
				}

				equipletInfo = new HashMap<AID, Pair<Double, Position>>();

				// option to execute product step :: Map < product step index, Map of options to execute product step <Equiplet, Pair < estimate duration of service, List of
				// possibilities < from time, until time> > > >
				Map<Integer, Map<AID, Pair<Double, List<Pair<Double, Double>>>>> options = new HashMap<Integer, Map<AID, Pair<Double, List<Pair<Double, Double>>>>>();

				System.out.printf("PA:%s waiting on answers of equiplets whether they can execute the product steps...\n", getLocalName());
				// receives the answers of the equiplets, whether they can execute the product steps
				int counter = 0;
				while (counter < suitedEquiplets.size()) {
					ACLMessage msg = receive();
					if (msg != null && msg.getPerformative() == ACLMessage.PROPOSE) {
						counter++;

						System.out.printf("PA:%s can execute reply received from %s : %s.\n", getLocalName(), msg.getSender().getLocalName(), msg.getContent());
						try {
							// answes = list of product steps < product step index, duration of service, list of possibilities >, load of the equiplet, and position of equiplet
							Triple<List<Triple<Integer, Double, List<Pair<Double, Double>>>>, Double, Position> answer = Parser.parseCanExecuteAnswer(msg.getContent());

							// add load and position of equiplet to equiplet info
							equipletInfo.put(msg.getSender(), new Pair<Double, Position>(answer.second, answer.third));

							// Triple < product step index, estimate production, possible times >
							for (Triple<Integer, Double, List<Pair<Double, Double>>> service : answer.first) {
								if (!options.containsKey(service.first)) {
									options.put(service.first, new HashMap<AID, Pair<Double, List<Pair<Double, Double>>>>());
								}

								options.get(service.first).put(msg.getSender(), new Pair<Double, List<Pair<Double, Double>>>(service.second, service.third));
							}
						} catch (JSONException e) {
							System.err.printf("PA:%s failed to receive correct message from equiplet %s when asking can execute %s.\n", getLocalName(), msg.getSender().getLocalName(), msg.getContent());
							System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
						}
					} else if (msg != null && msg.getPerformative() == ACLMessage.DISCONFIRM) {

					}
				}

				// calculate the best production path
				for (ProductStep productStep : productSteps) {
					if (options.containsKey(productStep.getIndex())) {

					} else {
						System.out.printf("PA:%s failed to find an equiplet that can execute the product step %s with the corresponding criteria.\n", getLocalName(), productStep);
						scheduled = false;
					}
				}

				/*
				 * Scheduling scheduling = new Scheduling(getLocalName(), getCreated(), deadline, position, productSteps, options, equipletInfo);
				 * LinkedList<Node> nodes = scheduling.calculateEDDPath();
				 * 
				 * boolean succeeded = true;
				 * if (nodes == null || nodes.size() != productSteps.size()) {
				 * System.out.println("P:" + getLocalName() + "  FAILED to find production path nodes=" + (nodes != null ? nodes : "null"));
				 * state = ProductState.ERROR;
				 * succeeded = false;
				 * } else {
				 * succeeded = schedule(nodes, deadline);
				 * scheduled = succeeded;
				 * 
				 * System.out.printf("PA:%s scheduled the following production path: %s.\n", getLocalName(), productionPath);
				 * 
				 * // if all succeed: product will travel to first equiplet
				 * 
				 * if (succeeded) {
				 * state = ProductState.TRAVELING;
				 * }
				 * }
				 * schedulingFinished(succeeded);
				 */

			}

			// Note: that retry is for service search and can execute question
			// maybe there should be distinction between the retries.
			retry--;
		}

		private HashMap<AID, LinkedList<ProductStep>> searchSuitedEquiplets() {
			HashMap<AID, LinkedList<ProductStep>> suitedEquiplets = new HashMap<>();
			for (ProductStep productStep : productSteps) {
				try {
					// Build the description used as template for the search
					DFAgentDescription template = new DFAgentDescription();
					ServiceDescription sd = new ServiceDescription();
					sd.setType(Ontology.SERVICE_SEARCH_TYPE);
					sd.setName(productStep.getService());

					template.addServices(sd);

					DFAgentDescription[] results = DFService.search(ProductAgent.this, template);
					if (results.length > 0) {
						for (int i = 0; i < results.length; i++) {
							AID equiplet = results[i].getName();

							if (!suitedEquiplets.containsKey(equiplet)) {
								suitedEquiplets.put(equiplet, new LinkedList<ProductStep>());
							}
							suitedEquiplets.get(equiplet).add(productStep);
						}
					} else {
						System.err.printf("PA:%s failed to find the service %s\n", getLocalName(), productStep.getService());
						return new HashMap<>();
					}
				} catch (FIPAException fe) {
					fe.printStackTrace();
					return new HashMap<>();
				}
			}
			return suitedEquiplets;
		}

		private boolean schedule(LinkedList<Node> nodes, double deadline) {
			LinkedList<ProductionStep> path = new LinkedList<>();
			HashMap<AID, ArrayList<ProductionStep>> sendList = new HashMap<>();
			for (int i = 0; i < nodes.size(); i++) {
				Node node = nodes.get(i);
				AID equiplet = node.getEquipletAID();
				ProductStep step = productSteps.get(i);
				ProductionStep production = new ProductionStep(step, equiplet, equipletInfo.get(equiplet).second, node.getTime(), node.getDuration());
				path.add(production);

				if (!sendList.containsKey(equiplet)) {
					sendList.put(equiplet, new ArrayList<ProductionStep>());
				}

				sendList.get(equiplet).add(production);
			}

			int sendCounter = 0;
			for (Entry<AID, ArrayList<ProductionStep>> entry : sendList.entrySet()) {
				try {
					// Ask the equiplet to schedule the service
					ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
					message.addReceiver(entry.getKey());
					message.setOntology(Ontology.GRID_ONTOLOGY);
					message.setConversationId(Ontology.CONVERSATION_SCHEDULE);
					message.setReplyWith(Ontology.CONVERSATION_SCHEDULE + System.currentTimeMillis());
					message.setContent(Parser.parseScheduleRequest(entry.getValue(), deadline));
					send(message);
					sendCounter++;
				} catch (JSONException e) {
					System.err.printf("PA:%s failed to construct message to equiplet %s for scheduling.\n", getLocalName(), entry.getKey());
					System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
				}
			}
			boolean succeeded = true;
			while (sendCounter > 0) { // All agent have to answer
				ACLMessage msg = blockingReceive(); // change to blockingReceive(millis) or blockingReceive(MessageTemplate, millis) to enable a timeout and or message for enable
													// more message handler
				if (msg != null && msg.getPerformative() == ACLMessage.CONFIRM) {
					sendCounter--;
				} else if (msg != null && msg.getPerformative() == ACLMessage.DISCONFIRM) {
					succeeded = false;
					sendCounter--;
				}
			}
			productionPath = path;

			return succeeded;
		}

		@Override
		public boolean done() {
			return (searched && scheduled) || retry < 1;
		}
	}

	public void schedulingFinished(boolean succeeded, LinkedList<ProductionStep> path) {
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
	public void onProductArrived(double time) {
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

	protected ProductionStep getCurrentStep() {
		return productionPath.peek();
	}
}
