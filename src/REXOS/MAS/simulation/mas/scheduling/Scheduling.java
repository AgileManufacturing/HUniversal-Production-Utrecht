package simulation.mas.scheduling;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Stack;

import simulation.mas.Equiplet;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.ProductStep;
import simulation.util.Tuple;

public class Scheduling {

	static boolean DEBUG_SCHEDULING = false;
	public static final int LOAD_WINDOW = 100;

	public LinkedList<Node> calculateEDDPath(double time, LinkedList<Pair<ProductStep, Map<String, Tuple<Double, Double, Double, Position>>>> productSteps, double deadline, double travelCost, Position start) {
		Graph<Node> graph = new Graph<>();

		Node source = new Node(time);
		Node sink = new Node();

		graph.add(source);
		graph.add(sink);

		// list of node in the last column
		Stack<Node> lastNodes = new Stack<Node>();
		lastNodes.add(source);

		// remember one iteration the Map of possible equiplets for the previous product step
		Map<String, Tuple<Double, Double, Double, Position>> previousSuitedEquiplets = null;

		// for each product step
		for (Pair<ProductStep, Map<String, Tuple<Double, Double, Double, Position>>> step : productSteps) {
			// keep track of the equiplets to process in the next iteration
			Stack<Node> equipletNodes = new Stack<Node>();

			// for each equiplet that can perform the product step
			// Entry < Equiplet name, Tuple < available, duration, load, equiplet position >  >
			for (Entry<String, Tuple<Double, Double, Double, Position>> entry : step.second.entrySet()) {

				Tuple<Double, Double, Double, Position> equiplet = entry.getValue();

				double available = equiplet.first;
				double duration = equiplet.second;
				// double load = equiplet.third;

				Position toPos = equiplet.fourth;

				// add a node with an arc to each node in the previous column 
				for (Node node : lastNodes) {
					Position fromPos = node != source ? previousSuitedEquiplets.get(node.getEquiplet()).fourth : start;

					double travel = calculateTravelTime(fromPos, toPos, travelCost);
					double arrival = node.getTime() + node.getDuration() + travel;
					double window = deadline - arrival;

					double firstPossibilty = Math.max(available, arrival) + duration;
					double cost = 1 - firstPossibilty / window;

					Node nextNode = new Node(entry.getKey(), firstPossibilty, duration);

					if (cost < 0 && DEBUG_SCHEDULING) {
						System.out.println("FAAAAAAAAAAAAAAAAAAIIIIIIIIIIIIIIIIIIIIILLLLLLLLLLLLLLLLLLl: " + deadline);
						System.out.printf("Add to graph: %s -- %.2f --> %s [cost=(1 - %.2f / %.2f)], available=%.2f, arrival=%.2f]\n", node, cost, nextNode, firstPossibilty, window, available, arrival);
					}

					graph.add(node, nextNode, cost);

					if (DEBUG_SCHEDULING) {
						System.out.printf("Add to graph: %s -- %.2f --> %s [cost=(1 - %.2f / %.2f)], available=%.2f, arrival=%.2f]\n", node, cost, nextNode, firstPossibilty, window, available, arrival);
					}

					equipletNodes.add(nextNode);
				}
			}

			if (DEBUG_SCHEDULING) {
				System.out.println("add nodes for " + step + " from : " + lastNodes + " to new equiplet nodes " + equipletNodes);
			}

			previousSuitedEquiplets = step.second;

			lastNodes.removeAllElements();
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
			System.out.println("Scheduling: Failed to find path in " + graph);
		}

		if (DEBUG_SCHEDULING) {
			System.out.println("the last equiplet nodes to be processed: " + lastNodes);
			System.out.println("Graph: " + graph);
		}

		return path;
	}

	private double calculateTravelTime(Position a, Position b, double travelCost) {
		return travelCost * Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
	}

	/**
	 * List < of product steps =
	 * - Pair< Product step to be executed for the product, List < of possible
	 * equiplet consisting of
	 * - - - Tuple<Equiplet name, available, duration, load> > > >
	 * 
	 * @param time
	 * @param productSteps
	 * @return
	 */
	public LinkedList<Node> calculatePath(double time, LinkedList<Pair<ProductStep, List<Tuple<String, Double, Double, Double>>>> productSteps) {
		// Create the graph to calculate best production path
		Graph<Node> graph = new Graph<Node>();

		// Add start and end to the graph
		Node source = new Node(time);
		Node sink = new Node();
		graph.add(source);
		graph.add(sink);

		// list of node in the last column
		Stack<Node> lastNodes = new Stack<Node>();
		lastNodes.add(source);

		for (Pair<ProductStep, List<Tuple<String, Double, Double, Double>>> step : productSteps) {
			// keep track of the equiplets to process in the next iteration
			Stack<Node> equipletNodes = new Stack<Node>();

			// add all the equiplet capable to execute the product step
			for (Tuple<String, Double, Double, Double> equiplet : step.second) {

				System.out.println("Process " + equiplet + " for product steps=" + step.first);

				for (Node lastNode : lastNodes) {
					// time the previous equiplet completes the production
					double stepComplete = lastNode.getTime() + lastNode.getDuration();

					// Calculate the cost for producing by the equiplet
					// travel times should be added if the previous time are greater the the travel + availible time
					double available = equiplet.second;

					// TODO delay is relative to the previous step, this might be not undesirable 
					// long delay = available > stepComplete + travel ? available - (stepComplete + travel) : 0;
					double delay = 1 - 1.0 * Math.max(0, available - stepComplete) / LOAD_WINDOW;
					double load = equiplet.fourth;

					double cost = cost(delay, load);
					Node node = new Node(equiplet.first, Math.max(available, stepComplete), equiplet.third);

					System.out.println("Add to graph: " + lastNode + " --" + cost + "--> " + node + " : cost(delay=" + delay + ", load=" + load + ") available=" + available + ", stepComplete=" + stepComplete);
					graph.add(lastNode, node, cost);

					equipletNodes.add(node);
				}
			}

			System.out.println("step: " + lastNodes + " - " + equipletNodes);

			lastNodes.removeAllElements();
			lastNodes.addAll(equipletNodes);
		}

		System.out.println("last step: " + lastNodes);

		for (Node node : lastNodes) {
			graph.add(node, sink, 0);
		}

		LinkedList<Node> path = graph.optimumPath(source, sink);
		if (path.size() > 1) {
			path.removeFirst();
			path.removeLast();
		} else if (path.isEmpty()) {
			System.out.println("Scheduling: Failed to find path in " + graph);
		}

		return path;
	}

	@Deprecated
	public LinkedList<Node> calculatePath_(double time, LinkedList<Pair<ProductStep, List<Pair<Equiplet, Double>>>> productSteps) {
		// Create the graph to calculate best production path
		Graph<Node> graph = new Graph<Node>();

		// Add start and end to the graph
		Node source = new Node(time);
		Node sink = new Node();
		graph.add(source);
		graph.add(sink);

		// list of node in the last column
		Stack<Node> lastNodes = new Stack<Node>();
		lastNodes.add(source);

		for (Pair<ProductStep, List<Pair<Equiplet, Double>>> step : productSteps) {
			// keep track of the equiplets to process in the next iteration
			Stack<Node> equipletNodes = new Stack<Node>();

			for (Pair<Equiplet, Double> equiplet : step.second) {

				for (Node lastNode : lastNodes) {
					// time the previous equiplet completes the production
					double stepComplete = lastNode.getTime() + lastNode.getDuration();

					// travel times should be added if the previous time are greater the the travel + availible time
					// TODO travel times from equiplet to equiplets
					double travel = 10; // .travelTime(lastNode.getEquiplet(), equiplet);

					// Calculate the cost for producing by the equiplet
					// TODO can this be answered before knowing the travel times?
					double available = equiplet.first.available(stepComplete + travel, step.first.getService());

					// TODO delay is relative to the previous step, this might be not undesirable 
					// long delay = available > stepComplete + travel ? available - (stepComplete + travel) : 0;
					double delay = 1 - 1.0 * Math.max(0, available + travel - stepComplete) / LOAD_WINDOW;
					double load = equiplet.first.load(time, LOAD_WINDOW);

					double cost = cost(delay, load);
					Node node = new Node(equiplet.first.getName(), Math.max(available, stepComplete), equiplet.second);

					System.out.println("Add to graph: " + lastNode + " --" + cost + "--> " + node + " : cost(delay=" + delay + ", load=" + load + ") available=" + available + ", stepComplete=" + stepComplete + ", travel=" + travel);
					graph.add(lastNode, node, cost);

					equipletNodes.add(node);
				}
			}

			System.out.println("step: " + lastNodes + " - " + equipletNodes);

			lastNodes.removeAllElements();
			lastNodes.addAll(equipletNodes);
		}

		System.out.println("last step: " + lastNodes);

		for (Node node : lastNodes) {
			graph.add(node, sink, 0);
		}

		LinkedList<Node> path = graph.optimumPath(source, sink);
		if (path.size() > 1) {
			path.removeFirst();
			path.removeLast();
		} else if (path.isEmpty()) {
			System.out.println("Scheduling: Failed to find path in " + graph);
		}

		return path;
	}

	private double cost(double delay, double load) {
		return delay * load;
	}
}
