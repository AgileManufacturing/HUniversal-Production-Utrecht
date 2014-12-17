package MAS.product;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.json.JSONException;

import MAS.util.Ontology;
import MAS.util.Pair;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.SchedulingAlgorithm;
import MAS.util.MASConfiguration;
import MAS.util.Tick;
import MAS.util.Triple;

public class ScheduleBehaviour extends OneShotBehaviour {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ProductAgent product;
	private LinkedList<ProductStep> productSteps;
	private Tick time;
	private Tick deadline;

	public ScheduleBehaviour(ProductAgent product, LinkedList<ProductStep> productSteps) {
		super(product);
		this.product = product;
		this.productSteps = productSteps;
		this.time = null;
		this.deadline = product.getDeadline();
	}

	public ScheduleBehaviour(ProductAgent product, LinkedList<ProductStep> steps, Tick time, Tick deadline) {
		super(product);
		this.product = product;
		this.productSteps = steps;
		this.time = time;
		this.deadline = deadline;
	}

	@Override
	public void action() {
		// when time is not specified, use product creation time
		time = time == null ? product.getCreated() : time;

		try {
			System.out.printf(System.currentTimeMillis() + "\tPA:%s starts schedule behaviour at %s, product steps: %s\n", myAgent.getLocalName(), time, productSteps);
			HashMap<AID, LinkedList<ProductStep>> suitedEquiplets = searchSuitedEquiplets(productSteps);

			System.out.printf(System.currentTimeMillis() + "\tPA:%s find the following suited equiplets %s\n", myAgent.getLocalName(), suitedEquiplets);

			Pair<Map<AID, Pair<Double, Position>>, Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>>> capableEquiplets = capableEquiplets(suitedEquiplets);
			Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> options = capableEquiplets.second;
			Map<AID, Pair<Double, Position>> equipletInfo = capableEquiplets.first;

			System.out.printf(System.currentTimeMillis() + "\tPA:%s filter the capable equiplets %s\n", myAgent.getLocalName(), equipletInfo.keySet());

			Map<Pair<Position, Position>, Tick> travelTimes = retrieveTravelTimes(product.getPosition(), options, equipletInfo);

			System.out.printf(System.currentTimeMillis() + "\tPA:%s retrieved travel times %s\n", myAgent.getLocalName(), travelTimes);

			Scheduling scheduling = new Scheduling(myAgent.getLocalName(), time, deadline, product.getPosition(), productSteps, options, equipletInfo, travelTimes);

			if (MASConfiguration.SCHEDULING == SchedulingAlgorithm.MATRIX) {
				LinkedList<ProductionStep> productionPath = scheduling.calculateMatrixPath();

				System.out.printf(System.currentTimeMillis() + "\tPA:%s path calculated %s\n", myAgent.getLocalName(), productionPath);

				schedule(productionPath, deadline);

				System.out.printf(System.currentTimeMillis() + "\tPA:%s scheduled equiplets.\n", myAgent.getLocalName());

				product.schedulingFinished(time, true, productionPath);
			} else {
				LinkedList<Node> nodes;
				if (MASConfiguration.SCHEDULING == SchedulingAlgorithm.EDD) {
					nodes = scheduling.calculateEDDPath();
				} else if (MASConfiguration.SCHEDULING == SchedulingAlgorithm.LOAD) {
					nodes = scheduling.calculateLoadPath();
				} else {
					nodes = scheduling.calculateSuprimePath();
				}

				System.out.printf(System.currentTimeMillis() + "\tPA:%s path calculated %s\n", myAgent.getLocalName(), nodes);

				LinkedList<ProductionStep> productionPath = schedule(nodes, productSteps, equipletInfo, deadline);
				System.out.printf(System.currentTimeMillis() + "\tPA:%s scheduled equiplets.\n", myAgent.getLocalName());
				product.schedulingFinished(time, true, productionPath);
			}

			System.out.printf(System.currentTimeMillis() + "\tPA:%s scheduling done.\n", myAgent.getLocalName());

		} catch (SchedulingException e) {
			System.out.printf("PA:%s scheduling failed: %s\n", myAgent.getLocalName(), e.getMessage());
			product.schedulingFinished(time, false);
		}
	}

	/**
	 * search for suited equiplet
	 * ask the DF if there are equiplets that have registered service needed for production of the product
	 * 
	 * @param productSteps
	 *            with the needed services
	 * @return a map equiplet that are suited for a set of product steps
	 * @throws SchedulingException
	 */
	private HashMap<AID, LinkedList<ProductStep>> searchSuitedEquiplets(List<ProductStep> productSteps) throws SchedulingException {
		// TODO communication improvement, instead of map equiplets to executable product step, change product step to service,
		// so it has to ask only once for a service instead of product step

		Map<String, List<AID>> searchedServices = new HashMap<>();
		HashMap<AID, LinkedList<ProductStep>> suitedEquiplets = new HashMap<>();
		for (ProductStep productStep : productSteps) {
			if (!searchedServices.containsKey(productStep.getService())) {
				try {
					// Build the description used as template for the search
					DFAgentDescription template = new DFAgentDescription();
					ServiceDescription sd = new ServiceDescription();
					sd.setType(Ontology.SERVICE_SEARCH_TYPE);
					sd.setName(productStep.getService());

					template.addServices(sd);

					DFAgentDescription[] results = DFService.search(myAgent, template);
					if (results.length > 0) {
						List<AID> equiplets = new ArrayList<>();
						for (int i = 0; i < results.length; i++) {
							AID equiplet = results[i].getName();

							if (!suitedEquiplets.containsKey(equiplet)) {
								suitedEquiplets.put(equiplet, new LinkedList<ProductStep>());
							}
							suitedEquiplets.get(equiplet).add(productStep);
							equiplets.add(equiplet);
						}
						searchedServices.put(productStep.getService(), equiplets);
					} else {
						System.err.printf("PA:%s failed to find the service %s\n", myAgent.getLocalName(), productStep.getService());
						return new HashMap<>();
					}
				} catch (FIPAException fe) {
					fe.printStackTrace();
					throw new SchedulingException(" failed to find services: " + fe.getMessage());
				}
			} else {
				for (AID equiplet : searchedServices.get(productStep.getService())) {
					suitedEquiplets.get(equiplet).add(productStep);
				}
			}
		}
		return suitedEquiplets;
	}

	/**
	 * filter incapable equiplets out of the suited equiplet list
	 * the equiplets will be contacted to retreive the availability, load and position of these equiplets
	 * 
	 * the answers will be sorted in a map of equiplet info and a map of options
	 * the equiplet info consists of the load and position for each equiplet
	 * the options consists a list of equiplets, with load, estimate for service and possibilities, for each product step index
	 * 
	 * @param suitedEquiplets
	 *            a map of equiplet with the suited product steps
	 * @return equiplet info and product step options
	 * @throws SchedulingException
	 *             when communication fails
	 */
	private Pair<Map<AID, Pair<Double, Position>>, Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>>> capableEquiplets(HashMap<AID, LinkedList<ProductStep>> suitedEquiplets)
			throws SchedulingException {
		String replyConversation = Ontology.CONVERSATION_CAN_EXECUTE + System.currentTimeMillis();

		// send a questions to each of the equilplet if the product step can be executed
		for (Entry<AID, LinkedList<ProductStep>> entry : suitedEquiplets.entrySet()) {
			try {
				ACLMessage message = new ACLMessage(ACLMessage.QUERY_REF);
				message.addReceiver(entry.getKey());
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_CAN_EXECUTE);
				message.setReplyWith(replyConversation);
				message.setContent(Parser.parseCanExecute(time, deadline, entry.getValue()));
				myAgent.send(message);
			} catch (JSONException e) {
				System.err.printf("PA:%s failed to construct message to equiplet for asking can execute %s.\n", myAgent.getLocalName(), entry.getValue());
				System.err.printf("PA:%s %s\n", myAgent.getLocalName(), e.getMessage());
			}
		}

		// equiplet info :: list of equiplet with the load and position of the equiplet
		Map<AID, Pair<Double, Position>> equipletInfo = new HashMap<>();

		// option to execute product step ::
		// Map < product step index, Options to execute product step <Equiplet, Triple < estimate duration of service, List of possibilities < from time, until time> > > >
		Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> serviceOptions = new HashMap<>();

		System.out.printf("PA:%s waiting on answers of equiplets whether they can execute the product steps...\n", myAgent.getLocalName());

		MessageTemplate template = MessageTemplate.MatchInReplyTo(replyConversation);

		// receives the answers of the equiplets, whether they can execute the product steps
		int counter = 0;
		while (counter < suitedEquiplets.size()) {
			ACLMessage msg = myAgent.blockingReceive(template, MASConfiguration.COMMUNICATION_TIMEOUT);

			// the equiplet is able to perform the product step and propose possibilities for executing the product step
			if (msg != null && msg.getPerformative() == ACLMessage.PROPOSE) {
				counter++;

				System.out.printf("PA:%s can execute reply received from %s : %s.\n", myAgent.getLocalName(), msg.getSender().getLocalName(), "");// msg.getContent());
				try {
					// Triple < List of product steps, load, position >
					Triple<List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>>, Double, Position> answer = Parser.parseCanExecuteAnswer(msg.getContent());
					equipletInfo.put(msg.getSender(), new Pair<Double, Position>(answer.second, answer.third));

					// Triple < product step index, estimate production, possible times >
					for (Triple<Integer, Tick, List<Pair<Tick, Tick>>> service : answer.first) {
						if (!serviceOptions.containsKey(service.first)) {
							serviceOptions.put(service.first, new HashMap<AID, Pair<Tick, List<Pair<Tick, Tick>>>>());
						}

						serviceOptions.get(service.first).put(msg.getSender(), new Pair<Tick, List<Pair<Tick, Tick>>>(service.second, service.third));
					}
				} catch (JSONException e) {
					System.err.printf("PA:%s failed to receive correct message from equiplet %s when asking can execute %s.\n", myAgent.getLocalName(), msg.getSender().getLocalName(), msg.getContent());
					System.err.printf("PA:%s %s\n", myAgent.getLocalName(), e.getMessage());
				}
			} else if (msg != null && msg.getPerformative() == ACLMessage.DISCONFIRM) {
				// equiplet is not able to execute the product step
			} else if (msg == null) {
				// TODO remove!
				throw new SchedulingException("message not received while waiting on equiplet reply if equiplet can execute product step");
			}
		}

		return new Pair<Map<AID, Pair<Double, Position>>, Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>>>(equipletInfo, serviceOptions);
	}

	/**
	 * retrieve the travel times from equiplet to equiplet by the traffic manager agent
	 * 
	 * @param position
	 *            of the product
	 * @param options
	 *            for executing a product step, for each product step a list of possible equiplets with estimate of duration and available times
	 * @param equipletInfo
	 *            information of the equiplet, load and position
	 * @return
	 * @throws SchedulingException
	 */
	private Map<Pair<Position, Position>, Tick> retrieveTravelTimes(Position position, Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> options, Map<AID, Pair<Double, Position>> equipletInfo)
			throws SchedulingException {
		// the routes of equiplet to equiplet for which the time is needed for scheduling
		Set<Pair<Position, Position>> routes = new HashSet<>();

		// constructing the routes, map for each product step the possible equiplet position to the next possibilities.
		// this creates a list of the edges the complete multipartite graph
		for (int i = productSteps.size() - 1; i > 0; i--) {
			// check if the product step is executable by one of the equiplets
			// note that at first there is no difference between the index of the product steps (i.e. productStep.get(i)) and the index of the product step (i.e.
			// productStep.get(i).getIndex()), when rescheduling the index do not match anymore
			int index = productSteps.get(i).getIndex();
			if (!options.containsKey(index)) {
				throw new SchedulingException("failed to find product step in options, i.e. there isn't a equiplet capable to execute the product step: " + productSteps.get(i));
			}
			if (!options.containsKey(index - 1)) {
				throw new SchedulingException("failed to find product step in options, i.e. there isn't a equiplet capable to execute the product step: " + productSteps.get(i - 1));
			}

			// System.out.println("[i=" + i + ", index=" + index + ", ps=" +productSteps.get(i) + ", options=" + options.get(i) + "]");
			// System.out.println("[i=" + i + ", index=" + index + ", ps=" +productSteps.get(i) + ", options=" + options.get(i) + "]");

			// map the possibilities for a product step to the next possibilities for the product step;
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> previousPossibilities = options.get(index - 1);
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> nextPossibilities = options.get(index);

			for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> previousEquiplet : previousPossibilities.entrySet()) {
				for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> nextEquiplet : nextPossibilities.entrySet()) {
					Position previousPos = equipletInfo.get(previousEquiplet.getKey()).second;
					Position nextPos = equipletInfo.get(nextEquiplet.getKey()).second;
					Pair<Position, Position> route = new Pair<>(previousPos, nextPos);
					if (previousPos != nextPos && !routes.contains(route)) {
						routes.add(route);
					}
				}
			}
		}

		// add routes from the current product position to the position of the possible equiplet for the first product step
		int firstIndex = productSteps.get(0).getIndex();
		if (options.containsKey(firstIndex)) {
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> possibilities = options.get(firstIndex);
			for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> equiplet : possibilities.entrySet()) {
				routes.add(new Pair<>(position, equipletInfo.get(equiplet.getKey()).second));
			}
		} else {
			throw new SchedulingException("failed to find product step in options, i.e. there isn't a equiplet capable to execute the product step: " + productSteps.get(0));
		}

		AID trafficAgent = new AID(MASConfiguration.TRAFFIC_AGENT, AID.ISLOCALNAME);
		String replyConversation = Ontology.CONVERSATION_TRAVEL_TIME + System.currentTimeMillis();
		try {
			ACLMessage message = new ACLMessage(ACLMessage.QUERY_REF);
			message.addReceiver(trafficAgent);
			message.setOntology(Ontology.GRID_ONTOLOGY);
			message.setConversationId(Ontology.CONVERSATION_TRAVEL_TIME);
			message.setReplyWith(replyConversation);
			message.setContent(Parser.parseTravelTimeRequest(routes));
			myAgent.send(message);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to construct message to the travel agent: %s for asking the travel time: %s\n", myAgent.getLocalName(), MASConfiguration.TRAFFIC_AGENT);
			System.err.printf("PA:%s %s\n", myAgent.getLocalName(), e.getMessage());
			throw new SchedulingException("failed retrieve travel times:" + e.getMessage());
		}

		System.out.printf("PA:%s asked the traffic agent for travel times and waiting on result\n", myAgent.getLocalName());

		// wait for reply
		try {
			MessageTemplate template = MessageTemplate.and(MessageTemplate.MatchSender(trafficAgent), MessageTemplate.MatchInReplyTo(replyConversation));

			ACLMessage reply = myAgent.blockingReceive(template, MASConfiguration.COMMUNICATION_TIMEOUT);
			if (reply != null) {
				Map<Pair<Position, Position>, Tick> travelTimes = Parser.parseTravelTimes(reply.getContent());

				System.out.printf("PA:%s received travel times %s\n", myAgent.getLocalName(), travelTimes);

				return travelTimes;
			} else {
				throw new SchedulingException("failed to receive travel times, message == null");
			}
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to construct message to the travel agent: %s for asking the travel time: %s\n", myAgent.getLocalName(), MASConfiguration.TRAFFIC_AGENT);
			System.err.printf("PA:%s %s\n", myAgent.getLocalName(), e.getMessage());
			throw new SchedulingException("failed retrieve travel times:" + e.getMessage());
		}
	}

	private LinkedList<ProductionStep> schedule(LinkedList<Node> nodes, LinkedList<ProductStep> productSteps, Map<AID, Pair<Double, Position>> equipletInfo, Tick deadline)
			throws SchedulingException {
		LinkedList<ProductionStep> path = new LinkedList<>();

		// construct send list grouped by equiplet
		HashMap<AID, ArrayList<ProductionStep>> sendList = new HashMap<>();
		for (int i = 0; i < nodes.size(); i++) {
			Node node = nodes.get(i);
			AID equiplet = node.getEquipletAID();

			ProductStep step = productSteps.get(i);
			Position equipletPosition = equipletInfo.get(equiplet).second;

			ProductionStep production = new ProductionStep(step, equiplet, equipletPosition, node.getTime(), node.getDuration());
			path.add(production);

			if (!sendList.containsKey(equiplet)) {
				sendList.put(equiplet, new ArrayList<ProductionStep>());
			}

			sendList.get(equiplet).add(production);
		}

		System.out.printf("PA:%s path to schedule: %s - send: %s\n", myAgent.getLocalName(), nodes, sendList);
		schedule(sendList, deadline);
		return path;
	}

	/**
	 * Schedule a list of production steps.
	 * This will grouped by equiplets and requested to be executed
	 * 
	 * @param path
	 *            of production steps
	 * @param deadline
	 *            of the product
	 * @throws SchedulingException
	 */
	private void schedule(LinkedList<ProductionStep> path, Tick deadline) throws SchedulingException {
		// construct send list grouped by equiplet
		HashMap<AID, ArrayList<ProductionStep>> sendList = new HashMap<>();
		for (ProductionStep production : path) {
			if (!sendList.containsKey(production.getEquiplet())) {
				sendList.put(production.getEquiplet(), new ArrayList<ProductionStep>());
			}
			sendList.get(production.getEquiplet()).add(production);
		}
		System.out.printf("PA:%s path to schedule: %s - send: %s\n", myAgent.getLocalName(), path, sendList);
		schedule(sendList, deadline);
	}

	/**
	 * Schedule a list of production steps grouped by equiplets
	 * 
	 * @param sendList
	 *            map of equiplets with the product step to schedule
	 * @param deadline
	 *            of the product
	 * @throws SchedulingException
	 */
	private void schedule(HashMap<AID, ArrayList<ProductionStep>> sendList, Tick deadline) throws SchedulingException {
		String replyConversation = Ontology.CONVERSATION_SCHEDULE + System.currentTimeMillis();
		int sendCounter = 0;
		for (Entry<AID, ArrayList<ProductionStep>> entry : sendList.entrySet()) {
			try {
				// Ask the equiplet to schedule the service
				ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
				message.addReceiver(entry.getKey());
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_SCHEDULE);
				message.setReplyWith(replyConversation);
				message.setContent(Parser.parseScheduleRequest(entry.getValue(), deadline));
				myAgent.send(message);
				sendCounter++;
			} catch (JSONException e) {
				System.err.printf("PA:%s failed to construct message to equiplet %s for scheduling.\n", myAgent.getLocalName(), entry.getKey());
				System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
				throw new SchedulingException("failed to construct message to equiplet for schedule product step");
			}
		}
		MessageTemplate template = MessageTemplate.MatchInReplyTo(replyConversation);
		while (sendCounter > 0) { // All agent have to answer
			ACLMessage msg = myAgent.blockingReceive(template, MASConfiguration.COMMUNICATION_TIMEOUT);
			if (msg != null && msg.getPerformative() == ACLMessage.CONFIRM) {
				sendCounter--;
			} else if (msg != null && msg.getPerformative() == ACLMessage.DISCONFIRM) {
				throw new SchedulingException("failed to get confirmation of all equiplets");
			} else if (msg == null) {
				throw new SchedulingException("failed schedule, message == null");
			}
		}
	}
}
