package simulation.mas.product;

import jade.core.AID;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Settings;
import simulation.util.Tuple;
import simulation.util.Util;

public class Scheduling {

	private String agent;
	private double time;
	private double deadline;
	private Position position;
	private List<ProductStep> productSteps;
	private Map<Integer, Map<AID, Pair<Double, List<Pair<Double, Double>>>>> serviceOptions;
	private Map<AID, Pair<Double, Position>> equipletInfo;
	private Map<Pair<Position, Position>, Double> travelTimes;

	public Scheduling(String agent, double time, double deadline, Position position, List<ProductStep> productSteps,
			Map<Integer, Map<AID, Pair<Double, List<Pair<Double, Double>>>>> options, Map<AID, Pair<Double, Position>> equipletInfo,
			Map<Pair<Position, Position>, Double> travelTimes) {
		this.agent = agent;
		this.time = time;
		this.deadline = deadline;
		this.position = position;
		this.productSteps = productSteps;
		this.serviceOptions = options;
		this.equipletInfo = equipletInfo;
		this.travelTimes = travelTimes;
	}

	public LinkedList<Node> calculateEDDPath() throws SchedulingException {
		Graph<Node> graph = new Graph<>();

		Node source = new Node(time);
		Node sink = new Node();

		graph.add(source);
		graph.add(sink);

		if (Settings.DEBUG_SCHEDULING) {
			System.out.printf("\nPA:%s calculate best path \ninfo: \t %s\noptions: \t%s\n", agent, Util.formatArray(equipletInfo), Util.formatArray(serviceOptions));
		}

		// list of node in the last column
		ArrayList<Node> lastNodes = new ArrayList<Node>();
		lastNodes.add(source);

		for (ProductStep step : productSteps) {
			Map<AID, Pair<Double, List<Pair<Double, Double>>>> options = serviceOptions.get(step.getIndex());

			if (Settings.DEBUG_SCHEDULING) {
				System.out.printf("\nPA:%s construct scheduling graph, step=%s, from nodes=%s, with options=%s.\n\n", agent, step, lastNodes, Util.formatArray(options));
			}

			// keep track of the equiplets to process in the next iteration
			ArrayList<Node> equipletNodes = new ArrayList<Node>();

			// add a node with an arc to each node in the previous column
			for (Node node : lastNodes) {

				// Entry < Equiplet, Pair < duration, List of possible time options > >
				for (Entry<AID, Pair<Double, List<Pair<Double, Double>>>> option : options.entrySet()) {
					Position lastPosition = node != source ? equipletInfo.get(node.getEquipletAID()).second : position;
					Position nextPosition = equipletInfo.get(option.getKey()).second;
					Pair<Position, Position> route = new Pair<>(lastPosition, nextPosition);
					if (!travelTimes.containsKey(route) && !lastPosition.equals(nextPosition)) {
						throw new SchedulingException("route doesn't exists in travel time list: " + route);
					}

					double window = deadline - time;
					double duration = option.getValue().first;
					double travel = lastPosition.equals(nextPosition) ? 0.0 : travelTimes.get(new Pair<>(lastPosition, nextPosition));
					double arrival = node.getTime() + node.getDuration() + travel;
					double firstPossibilty = Double.MAX_VALUE;

					// Time option is the time from (=option.first) until (=option.second) the equiplet is possible to perform the service
					for (Pair<Double, Double> timeOption : option.getValue().second) {
						
						
						
						// choose the best time to perform the product step
						if (timeOption.first < firstPossibilty && arrival < timeOption.second) {

							// TODO performance improvement
							// set the first possibility, the first is the time the equiplet is able to perform or when the product can arrive by the equiplet
							firstPossibilty = Math.max(timeOption.first, arrival);
						}
					}

					// check if deadline can be reached
					if (firstPossibilty + duration <= deadline) {
						Node nextNode = new Node(option.getKey(), firstPossibilty, duration);

						double cost = 1 - (firstPossibilty - time) / window;

						if (cost < 0) { // ) && Settings.DEBUG_SCHEDULING) {
							// shouldn't occur as it would mean arrival of first possibility > deadline
							System.err.println("FAILED maybe because the: deadline=" + deadline);
							System.err.printf("Should happen: Add to graph: (%s) -- %.6f --> (%s) [cost=(1 - %.2f / %.2f)], arrival=%.2f]\n", node, cost, nextNode, firstPossibilty, window, arrival);
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
		} else if (path.isEmpty() || path.size() != productSteps.size()) {
			throw new SchedulingException("failed to find path int nodes=" + graph + " - " + path);
		}

		if (Settings.DEBUG_SCHEDULING) {
			System.out.println("the last equiplet nodes to be processed: " + lastNodes);
			System.out.println("Graph: " + graph);
		}

		return path;
	}

	public LinkedList<Node> calculateMatrixPath() {
		return null;
	}

	/**
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more information see
	 * 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user requirements to product' - Leo van Moergestel section 3.2
	 * 
	 * @param equiplets
	 *            Map < Equiplet, < List<Service able to perform>, load of equiplet>
	 * @param productSteps
	 *            list of product steps of the product
	 * @return the generated scheduleMatrix
	 */
	private double[][] generateMatrix(ArrayList<Tuple<AID, List<String>, Double, Double>> equiplets, ArrayList<ProductStep> productSteps) {
		double[][] matrix = new double[equiplets.size()][productSteps.size()];
		// initialize matrix
		for (int r = 0; r < matrix.length; r++) {
			for (int c = 0; c < matrix.length; c++) {
				matrix[r][c] = 1.0;
			}
		}

		// iterate over the equiplets, the rows of the matrix
		for (int row = 0; row < equiplets.size(); row++) {
			List<String> capableServices = equiplets.get(row).second;
			double load = equiplets.get(row).third;

			// always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
			// the sequenceLength is the number of the product steps that can be consecutive performed by an equiplet
			int sequenceLength = 0;

			// the first product step in the sequence that an equiplet can perform
			int firstInSequence = -1;

			// iterate over the product steps, the columns of the matrix
			for (int column = 0; column < productSteps.size(); column++) {
				ProductStep productStep = productSteps.get(column);

				// if equiplet can perform product step
				// TODO match criteria if needed
				if (capableServices.contains(productStep.getService())) {
					// set the first item in the sequence.
					if (firstInSequence < 0) {
						firstInSequence = column;
					}
					sequenceLength++;

					if (column == productSteps.size() - 1) { // end of row
						// set value of sequence
						for (int c = firstInSequence; c < matrix[row].length; c++) {
							int value = sequenceLength - 1;
							matrix[row][c] = value;
						}
					}
				} else if (sequenceLength > 0) {
					// end of sequence, set value of sequence
					for (int c = firstInSequence; c < matrix[row].length; c++) {
						int value = sequenceLength - 1;
						matrix[row][c] = value;
					}
					sequenceLength = 0;
					firstInSequence = -1;
				}

				// value might have changed since we added sequence multiplier
				// TODO: perform supermagic calculation of currentValue * load here
				for (int c = firstInSequence; c < matrix[row].length; c++) {
					int value = sequenceLength - 1;
					matrix[row][c] = value;
				}
				matrix[row][column] = matrix[row][column] * load;
			}
		}

		return matrix;
	}
}
