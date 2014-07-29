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

import simulation.simulation.ISimulation;
import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.Settings;
import simulation.util.Triple;
import simulation.util.Util;

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
	private ISimulation simulation;

	public ProductAgent(ISimulation simulation, LinkedList<ProductStep> productSteps, Position startPosition, double time) {
		this.simulation = simulation;
		this.created = time;
		setup(productSteps, startPosition);
	}

	public void setup() {
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			try {
				Pair<List<ProductStep>, Position> configuration = Parser.parseProductConfiguration(args[0].toString());
				setup(configuration.first, configuration.second);
				this.created = Double.valueOf(System.currentTimeMillis());

			} catch (JSONException e) {
				System.err.printf("PA:%s failed to parse the arguments\n", getLocalName());
				System.err.printf("PA:%s %s", getLocalName(), e.getMessage());
				state = ProductState.ERROR;
			}
		} else if (simulation == null) {
			System.err.printf("PA:%s Failed to receive correct arguments\n", getLocalName());
			state = ProductState.ERROR;
		}
	}

	public void setup(List<ProductStep> productSteps, Position startPosition) {
		this.position = startPosition;
		this.productSteps = productSteps;
		this.productionPath = new LinkedList<>();

		this.deadline = created + 10000;
		this.state = ProductState.SCHEDULING;

		System.out.printf("PA:%s initialize [pos=%s, product steps=%s, deadline=%.0f]\n", getLocalName(), position, productSteps, deadline);

		addBehaviour(new ScheduleBehaviour());
		addBehaviour(new ProductListenerBehaviour());
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

	public class ScheduleBehaviour extends Behaviour {

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

		public ScheduleBehaviour() {
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
						message.setContent(Parser.parseCanExecute(created, deadline, entry.getValue()));
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
							// Triple < List of product steps, load, position >
							Triple<List<Triple<Integer, Double, List<Pair<Double, Double>>>>, Double, Position> answer = Parser.parseCanExecuteAnswer(msg.getContent());
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

				boolean succeeded = true;
				LinkedList<Node> nodes = calculateEDDPath(created, deadline, position, productSteps, options);
				if (nodes == null || nodes.size() != productSteps.size()) {
					System.out.println("P:" + getLocalName() + "  FAILED to find production path nodes=" + (nodes != null ? nodes : "null"));
					state = ProductState.ERROR;
					succeeded = false;

					// let the simulation know that the creation of product agent failed
					simulation.notifyProductCreationFailed(getLocalName());
				} else {
					succeeded = schedule(nodes);
					scheduled = succeeded;

					System.out.printf("PA:%s scheduled the following production path: %s.\n", getLocalName(), productionPath);

					// if all succeed: product will travel to first equiplet
					if (succeeded) {
						simulation.notifyProductCreated(getLocalName(), productionPath.peek().getEquipletName());
						state = ProductState.TRAVELING;
					} else {
						simulation.notifyProductCreationFailed(getLocalName());
					}
				}

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

		private LinkedList<Node> calculateEDDPath(double time, double deadline, Position position, List<ProductStep> productSteps, Map<Integer, Map<AID, Pair<Double, List<Pair<Double, Double>>>>> serviceOptions) {
			Graph<Node> graph = new Graph<>();

			Node source = new Node(time);
			Node sink = new Node();

			graph.add(source);
			graph.add(sink);

			if (Settings.DEBUG_SCHEDULING) {
				System.out.printf("\nPA:%s calculate best path \ninfo: \t %s\noptions: \t%s\n", getLocalName(), Util.formatArray(equipletInfo), Util.formatArray(serviceOptions));
			}

			// list of node in the last column
			ArrayList<Node> lastNodes = new ArrayList<Node>();
			lastNodes.add(source);

			for (ProductStep step : productSteps) {
				Map<AID, Pair<Double, List<Pair<Double, Double>>>> options = serviceOptions.get(step.getIndex());

				if (Settings.DEBUG_SCHEDULING) {
					System.out.printf("\nPA:%s construct scheduling graph, step=%s, from nodes=%s, with options=%s.\n\n", getLocalName(), step, lastNodes, Util.formatArray(options));
				}

				// keep track of the equiplets to process in the next iteration
				ArrayList<Node> equipletNodes = new ArrayList<Node>();

				// add a node with an arc to each node in the previous column
				for (Node node : lastNodes) {

					// Entry < Equiplet, Pair < duration, List of possible time options > >
					for (Entry<AID, Pair<Double, List<Pair<Double, Double>>>> option : options.entrySet()) {

						Position lastPosition = node != source ? equipletInfo.get(node.getEquipletAID()).second : position;
						Position nextPosition = equipletInfo.get(option.getKey()).second;

						double duration = option.getValue().first;
						double travel = caclulateTravelTime(lastPosition, nextPosition);
						double arrival = node.getTime() + node.getDuration() + travel;
						double firstPossibilty = Double.MAX_VALUE;

						// Time option is the time from : first until : second the equiplet is possible to perform the service
						for (Pair<Double, Double> timeOption : option.getValue().second) {
							// choose the best time to perform the product step
							if (timeOption.first < firstPossibilty && arrival < timeOption.second) {

								// TODO performance improvement
								firstPossibilty = Math.max(timeOption.first, arrival); // set the first possibility, the first is the time the equiplet is able to perform or when
																						// the
																						// product can arrive by the equiplet
							}
						}

						// check if deadline can be reached
						if (arrival < deadline) {

							Node nextNode = new Node(option.getKey(), firstPossibilty, duration);

							double window = deadline - arrival;
							double cost = 1 - (firstPossibilty - created) / window;

							if (cost < 0) { // ) && Settings.DEBUG_SCHEDULING) {
								// shouldn't occur as it would mean arrival > deadline 
								System.out.println("FAILED maybe because the: deadline=" + deadline);
								System.out.printf("Add to graph: (%s) -- %.6f --> (%s) [cost=(1 - %.2f / %.2f)], arrival=%.2f]\n", node, cost, nextNode, firstPossibilty, window, arrival);
							}

							graph.add(node, nextNode, cost);
							equipletNodes.add(nextNode);

							if (Settings.DEBUG_SCHEDULING) {
								System.out.printf("Add to graph: (%s) -- %.6f --> (%s) [cost=(1 - %.2f / %.2f)], arrival=%.2f]\n", node, cost, nextNode, firstPossibilty, window, arrival);
							}
						}
					}
				}

				lastNodes.clear();
				lastNodes.addAll(equipletNodes);
			}

			// add vertces from all the nodes in the last column to the sink node
			for (Node node : lastNodes) {
				graph.add(node, sink, 0);
			}

			LinkedList<Node> path = graph.optimumPath(source, sink);
			if (path.size() > 1) {
				path.removeFirst();
				path.removeLast();
			} else if (path.isEmpty()) {
				System.err.printf("PA:%s Failed to find path in %s\n", getLocalName(), graph);
			}

			if (Settings.DEBUG_SCHEDULING) {
				System.out.println("the last equiplet nodes to be processed: " + lastNodes);
				System.out.println("Graph: " + graph);
			}

			return path;
		}

		private boolean schedule(LinkedList<Node> nodes) {
			int sendCounter = 0;
			LinkedList<ProductionStep> path = new LinkedList<>();
			for (int i = 0; i < nodes.size(); i++) {
				Node node = nodes.get(i);
				ProductStep step = productSteps.get(i);
				AID equiplet = node.getEquipletAID();
				path.add(new ProductionStep(step, equiplet, equipletInfo.get(equiplet).second, node.getTime(), node.getDuration()));

				String service = step.getService();
				Map<String, Object> criteria = step.getCriteria();

				try {
					// Ask the equiplet to schedule the service
					ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
					message.addReceiver(equiplet);
					message.setOntology(Ontology.GRID_ONTOLOGY);
					message.setConversationId(Ontology.CONVERSATION_SCHEDULE);
					message.setReplyWith(Ontology.CONVERSATION_SCHEDULE + System.currentTimeMillis());
					message.setContent(Parser.parseSchedule(service, criteria, node.getTime(), deadline));
					send(message);
					sendCounter++;
				} catch (JSONException e) {
					System.err.printf("PA:%s failed to construct message to equiplet %s for scheduling.\n", getLocalName(), equiplet);
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

	private double caclulateTravelTime(Position a, Position b) {
		int travelSquares = Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
		return 0.5 * travelSquares;
	}

	protected ProductState getProductState() {
		return state;
	}

	public Position getPosition() {
		return position;
	}

	@Override
	public String toString() {
		return String.format("Product: %s [state=%s, created=%.2f, position=%s, current step=%s, product steps=%s, path=%s]", getLocalName(), state, created, position, (productionPath.size() > 0 ? productionPath.peek()
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

			// notify simulation product finished
			simulation.notifyProductFinished(getLocalName());
		} else {
			state = ProductState.TRAVELING;

			simulation.notifyProductTraveling(getLocalName(), productionPath.peek().getEquipletName());
		}
	}

	protected void onProductProcessing() {
		state = ProductState.PROCESSING;

		// notify the simulation that processing begins
		simulation.notifyProductProcessing(getLocalName(), productionPath.peek().getEquipletName(), productionPath.peek().getService());
	}

	private ProductionStep getCurrentStep() {
		return productionPath.peek();
	}

	public double getCreated() {
		return created;
	}
}
