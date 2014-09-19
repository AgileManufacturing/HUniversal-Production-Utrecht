package MAS.simulation.mas.product;

import jade.core.AID;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import MAS.simulation.util.Pair;
import MAS.simulation.util.Position;
import MAS.simulation.util.Settings;
import MAS.simulation.util.Tick;
import MAS.simulation.util.Tuple;
import MAS.simulation.util.Util;

public class Scheduling {

	private String agent;
	private Tick time;
	private Tick deadline;
	private Position position;
	private List<ProductStep> productSteps;
	private Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> serviceOptions;
	private Map<AID, Pair<Double, Position>> equipletInfo;
	private Map<Pair<Position, Position>, Tick> travelTimes;

	public Scheduling(String agent, Tick time, Tick deadline, Position position, List<ProductStep> productSteps,
			Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> options, Map<AID, Pair<Double, Position>> equipletInfo, Map<Pair<Position, Position>, Tick> travelTimes) {
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

		if (Settings.VERBOSITY > 3) {
			System.out.printf("\nPA:%s calculate best path \ninfo: \t %s\noptions: \t%s\n", agent, Util.formatArray(equipletInfo), Util.formatArray(serviceOptions));
		}

		// list of node in the last column
		ArrayList<Node> lastNodes = new ArrayList<Node>();
		lastNodes.add(source);

		for (ProductStep step : productSteps) {
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> options = serviceOptions.get(step.getIndex());

			if (Settings.VERBOSITY > 3) {
				System.out.printf("\nPA:%s construct scheduling graph, step=%s, from nodes=%s, with options=%s.\n\n", agent, step, lastNodes, Util.formatArray(options));
			}

			// keep track of the equiplets to process in the next iteration
			ArrayList<Node> equipletNodes = new ArrayList<Node>();

			// add a node with an arc to each node in the previous column
			for (Node node : lastNodes) {

				// Entry < Equiplet, Pair < duration, List of possible time options > >
				for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> option : options.entrySet()) {
					Position lastPosition = node != source ? equipletInfo.get(node.getEquipletAID()).second : position;
					Position nextPosition = equipletInfo.get(option.getKey()).second;
					Pair<Position, Position> route = new Pair<>(lastPosition, nextPosition);

					// check if the travel time from the route is know, and is not the the same as the previous
					if (!travelTimes.containsKey(route) && !lastPosition.equals(nextPosition)) {
						throw new SchedulingException("route doesn't exists in travel time list: " + route);
					}

					Tick window = deadline.minus(time);
					Tick duration = option.getValue().first;
					Tick travel = lastPosition.equals(nextPosition) ? new Tick(0) : travelTimes.get(new Pair<>(lastPosition, nextPosition));
					Tick arrival = node.getTime().add(node.getDuration()).add(travel);
					Tick firstPossibility = deadline;

					// Time option is the time from (=option.first) until (=option.second) the equiplet is possible to perform the service
					for (Pair<Tick, Tick> timeOption : option.getValue().second) {

						// choose the best time to perform the product step
						// is the first available time earlier than first possibility and the product can arrive + duration is within the time window
						if (timeOption.first.lessThan(firstPossibility) && arrival.add(duration).lessThan(timeOption.second)) {

							// TODO performance improvement
							// set the first possibility, the first is the time the equiplet is able to perform or when the product can arrive by the equiplet
							firstPossibility = timeOption.first.max(arrival);
						}
					}

					// check if deadline can be reached
					if (firstPossibility.add(duration).lessOrEqualThan(deadline)) {
						Node nextNode = new Node(option.getKey(), firstPossibility, duration, step.getIndex());

						double cost = 1 - firstPossibility.minus(time).doubleValue() / window.doubleValue();

						if (cost < 0) { // ) && Settings.VERBOSITY > 3) {
							// shouldn't occur as it would mean arrival of first possibility > deadline
							System.err.println("FAILED maybe because the: deadline=" + deadline);
							System.err.printf("Should happen: Add to graph: (%s) -- %.6f --> (%s) [cost=(1 - %s / %s)], arrival=%s]\n", node, cost, nextNode, firstPossibility, window, arrival);
						}

						graph.add(node, nextNode, cost);
						equipletNodes.add(nextNode);

						if (Settings.VERBOSITY > 3) {
							System.out.printf("Add to graph: (%s) -- %.6f --> (%s) [cost=(1 - %s / %s)], arrival=%s]\n", node, cost, nextNode, firstPossibility, window, arrival);
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

		if (Settings.VERBOSITY > 3) {
			System.out.println("the last equiplet nodes to be processed: " + lastNodes);
			System.out.println("Graph: " + graph);
		}

		return path;
	}

	public LinkedList<ProductionStep> calculateMatrixPath() throws SchedulingException {
		double[][] matrix = new double[equipletInfo.size()][productSteps.size()];
		List<AID> equiplets = new ArrayList<>(equipletInfo.keySet());

		for (int row = 0; row < equiplets.size(); row++) {
			AID equiplet = equiplets.get(row);

			// always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
			// the sequenceLength is the number of the product steps that can be consecutive performed by an equiplet
			int sequenceLength = 0;

			// the first product step in the sequence that an equiplet can perform
			int firstInSequence = -1;

			for (int column = 0; column < productSteps.size(); column++) {

				// check if the equiplet is capable to execute the product step
				if (serviceOptions.get(column).containsKey(equiplet)) {
					// set the first item in the sequence.
					if (firstInSequence < 0) {
						firstInSequence = column;
					}
					sequenceLength++;

					// set the values of the matrix when the end of a row is reached
					if (column == productSteps.size() - 1) {
						// set value of sequence
						for (int c = firstInSequence; c < matrix[row].length; c++) {
							matrix[row][c] = sequenceLength;
						}
					}
				} else {
					matrix[row][column] = 0;

					// set the previous values in the matrix when there is a sequence
					if (firstInSequence >= 0) {
						for (int c = firstInSequence; c < firstInSequence + sequenceLength; c++) {
							matrix[row][c] = sequenceLength;
						}
						sequenceLength = 0;
						firstInSequence = -1;
					}
				}
			}
		}

		if (Settings.VERBOSITY > 3) {
			System.out.println("Scheduling Matrix: step " + productSteps + " \n" + Util.formatMatrix(matrix));
		}

		Pair<Position, Tick> previousStep = new Pair<Position, Tick>(position, time);

		LinkedList<ProductionStep> path = new LinkedList<>();
		for (int column = 0; column < productSteps.size(); column++) {

			double highScore = -1;
			int highestRow = -1;
			for (int row = 0; row < equipletInfo.size(); row++) {
				if (matrix[row][column] > 0 && matrix[row][column] > highScore) {
					highScore = matrix[row][column];
					highestRow = row;
				}
			}

			// private Map<Integer, Map<AID, Pair<Double, List<Pair<Double, Double>>>>> serviceOptions;
			if (highestRow >= 0) {
				AID equiplet = equiplets.get(highestRow);
				Pair<Tick, List<Pair<Tick, Tick>>> option = serviceOptions.get(column).get(equiplet);
				List<Pair<Tick, Tick>> availableTimeSlots = option.second;

				Tick travelTime = previousStep.first.equals(equipletInfo.get(equiplet).second) ? new Tick(0)
						: travelTimes.get(new Pair<Position, Object>(previousStep.first, equipletInfo.get(equiplet).second));
				Tick arrival = previousStep.second.add(travelTime);
				Tick duration = option.first;
				Tick firstPossibility = deadline;

				for (Pair<Tick, Tick> timeSlot : availableTimeSlots) {
					if (timeSlot.first.lessThan(firstPossibility) && arrival.add(duration).lessOrEqualThan(timeSlot.second)) {
						firstPossibility = timeSlot.first.max(arrival);
					}
				}

				if (Settings.VERBOSITY > 3) {
					System.out.println("for equiplet " + equiplet.getLocalName() + "(" + productSteps.get(column).getService() + ") \tscoring=" + highScore
							+ " , can arrive at (pre=" + previousStep.second + " + " + travelTime + ")=" + arrival + ", duration=" + duration + ", available time:"
							+ availableTimeSlots + ", so first possibility=" + firstPossibility);
				}

				if (firstPossibility.lessThan(deadline)) {
					path.add(new ProductionStep(productSteps.get(column), equiplet, equipletInfo.get(equiplet).second, firstPossibility, deadline));
					previousStep = new Pair<Position, Tick>(equipletInfo.get(equiplet).second, firstPossibility.add(duration));
				} else {
					// throw new SchedulingException("You selected the wrong equiplet, TODO shit because no available time is found ");
					throw new SchedulingException("not able to find equiplet available before deadline");
				}

			} else {
				throw new SchedulingException("failed to schedule product steps, for product step " + productSteps.get(column) + " is no capable equiplet found.");
			}
		}

		return path;
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
	@SuppressWarnings("unused")
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
