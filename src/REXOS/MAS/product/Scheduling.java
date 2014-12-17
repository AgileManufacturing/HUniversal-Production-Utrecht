package MAS.product;

import jade.core.AID;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeSet;

import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.MASConfiguration;
import MAS.util.Tick;
import MAS.util.Tuple;
import MAS.util.Util;

public class Scheduling {

	private String agent;
	private Tick time;
	private Tick deadline;
	private Position position;
	private List<ProductStep> productSteps;

	// service options :: Map < product step index, [ options :: <Equiplet, Pair < estimate duration of service, [ possibilities :: < from time, until time > ] > > ] >
	private Map<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> serviceOptions;

	// equiplet info :: list of equiplet with the load and position of the equiplet
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
		
		if (MASConfiguration.VERBOSITY > 3) {
			System.out.println("product steps: " + productSteps);
			System.out.println("service options: " + Util.formatArray(serviceOptions));
			System.out.println("equiplet info: " + Util.formatArray(equipletInfo));
			System.out.println("travel times: " + Util.formatArray(travelTimes));
		}
	}

	public void validateAlgorithm() throws SchedulingException {
		LinkedList<Node> optimumPath = calculateOptimumLoadPath();
		LinkedList<Node> loadPath = calculateLoadPath();

		// shit should not be equal but is loads are the same for equiplet with the
		// if loads are equal of equiplets with same capabilities, shit goes wrong a bit random.
		boolean shitCanFail = false;
		for (Entry<Integer, Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>>> options : serviceOptions.entrySet()) {
			double lowestLoad = Double.MAX_VALUE;
			double counter = 0;

			for (AID equiplet : options.getValue().keySet()) {
				double load = equipletInfo.get(equiplet).first;
				if (load < lowestLoad) {
					lowestLoad = load;
					counter = 1;
				} else if (load == lowestLoad) {
					counter++;
				}
			}
			if (counter > 1) {
				shitCanFail = true;
			}
		}

		// all option should be explored
		if (MASConfiguration.VERBOSITY > 3) {
			System.out.println("Scheduling: found path with possible best score: " + loadPath + " comparing=" + loadPath.equals(optimumPath) + ", can fail=" + shitCanFail);
		}

		if (!loadPath.equals(optimumPath) && shitCanFail) {
			throw new SchedulingException("Scheduling: should found the same path or maybe not: " + loadPath + " == " + optimumPath);
		}
	}

	/**
	 * bad performance
	 * 
	 * @return
	 * @throws SchedulingException
	 */
	public LinkedList<Node> calculateOptimumEDDPath() throws SchedulingException {
		Graph<Node> graph = new Graph<>();

		Node source = new Node(time);
		Node sink = new Node();

		graph.add(source);
		graph.add(sink);

		if (MASConfiguration.VERBOSITY > 3) {
			System.out.printf("\nPA:%s calculate best path \ninfo: \t %s\noptions: \t%s\n", agent, Util.formatArray(equipletInfo), Util.formatArray(serviceOptions));
		}

		// list of node in the last column
		ArrayList<Node> lastNodes = new ArrayList<Node>();
		lastNodes.add(source);

		for (ProductStep step : productSteps) {
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> options = serviceOptions.get(step.getIndex());

			if (MASConfiguration.VERBOSITY > 3) {
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

						if (equipletNodes.contains(nextNode)) {
							int index = equipletNodes.indexOf(nextNode);
							Node similarNode = equipletNodes.get(index);

							// WTF this should work, but life sucks...
							// if (nextNode.getTime().lessThan(similarNode.getTime())) {
							if (nextNode.getTime().doubleValue() < similarNode.getTime().doubleValue()) {
								equipletNodes.remove(index);
								equipletNodes.add(nextNode);
							}
						} else {
							equipletNodes.add(nextNode);
						}

						if (MASConfiguration.VERBOSITY > 3) {
							// System.out.printf("Add to graph: (%s) -- %.6f --> (%s) [cost=(1 - %s / %s)], arrival=%s]\n", node, cost, nextNode, firstPossibility, window,
							// arrival);
						}
					}
				}
			}

			System.out.println(" equiplet nodes " + equipletNodes);

			lastNodes.clear();
			lastNodes.addAll(equipletNodes);

			System.out.println("Graph so far:");
			System.out.println("Graph: " + graph);
			System.out.println("\nGraph: " + graph.prettyPrint(source));
			// System.out.println();
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

		if (MASConfiguration.VERBOSITY > 3) {
			System.out.println("the last equiplet nodes to be processed: " + lastNodes);
			System.out.println("Graph: " + graph);
		}

		return path;
	}

	/**
	 * This algorithm searches for the path with the equiplets with the least
	 * load. It will construct a Graph G = (N, A).
	 * If the options of each product steps are the same: the graph will consist
	 * of (2 + # product steps * # options per product step) nodes and (2 * #
	 * product steps + # product steps * # options per product step ^ 2)
	 * This has bad performance if there are many options for each product step
	 * and even lead to: OutOfMemoryError exception: Java heap space.
	 * the reason of the memory problems are that each path will be calculated.
	 * #options per product step ^ # product steps
	 * The function should only be used for debug purposes.
	 * 
	 * @return path
	 * @throws SchedulingException
	 *             if no path will can be found within the deadline
	 */
	public LinkedList<Node> calculateOptimumLoadPath() throws SchedulingException {
		Node source = new Node(time);
		Node sink = new Node();

		Graph<Node> graph = new Graph<>();
		graph.add(source);
		graph.add(sink);

		// list of node in the last column
		ArrayList<Node> lastNodes = new ArrayList<Node>();
		lastNodes.add(source);

		for (ProductStep step : productSteps) {
			Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> options = serviceOptions.get(step.getIndex());

			// keep track of the equiplets to process in the next iteration
			ArrayList<Node> equipletNodes = new ArrayList<Node>();

			// add a node with an arc to each node in the previous column
			for (Node previousNode : lastNodes) {

				// Entry < Equiplet, Pair < duration, List of possible time options > >
				for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> option : options.entrySet()) {
					AID equiplet = option.getKey();
					Tick duration = option.getValue().first;

					Node nextNode = new Node(equiplet, new Tick(1), duration, step.getIndex());
					double load = equipletInfo.get(equiplet).first;

					boolean added = graph.add(previousNode, nextNode, load);

					if (added) {
						equipletNodes.add(nextNode);
					}
				}
			}

			lastNodes.clear();
			lastNodes.addAll(equipletNodes);

			// System.out.println("Graph so far:");
			// System.out.println("Graph: " + graph);
			// System.out.println("\nGraph pretty: " + graph.prettyPrint(source));
		}

		// add vertces from all the nodes in the last column to the sink node
		for (Node node : lastNodes) {
			graph.add(node, sink, 0);
		}

		// path through graph calculation
		LinkedList<Node> optimumPath = graph.optimumPath(source, sink);
		if (optimumPath.size() > 1) {
			optimumPath.removeFirst();
			optimumPath.removeLast();
		} else if (optimumPath.isEmpty() || optimumPath.size() != productSteps.size()) {
			throw new SchedulingException("failed to find path int nodes=" + graph + " - " + optimumPath);
		}

		if (MASConfiguration.VERBOSITY > 3) {
			System.out.println("the last equiplet nodes to be processed: " + lastNodes);
			System.out.println("\nGraph pretty: " + graph.prettyPrint(source));
			System.out.println("Graph: " + graph);
			System.out.println(" Optimum Path : " + optimumPath);
		}

		return optimumPath;
	}

	/**
	 * Calculate a good path on the basis of the load of equiplets. This will
	 * explore only the options if they remain with the best T options,
	 * therefore it can not ensure the optimum path is found.
	 * 
	 * The algorithm keep a sorted list, on the basis of score, of the
	 * calculated sub paths. While there are paths and #elements in the path are
	 * smaller than the #product steps: the best path so far will be
	 * removed and for each neighbor a new sub path added.
	 * 
	 * @return best path
	 * @throws SchedulingException
	 */
	public LinkedList<Node> calculateLoadPath() throws SchedulingException {
		return calculatePath(new Score() {
			@Override
			double score(Tick possibility, double load) {
				return load;
			}

		}, 100);
	}

	public LinkedList<Node> calculateEDDPath() throws SchedulingException {
		return calculatePath(new Score() {
			@Override
			double score(Tick possibility, double load) {
				return deadline.minus(possibility).doubleValue() / deadline.minus(time).doubleValue();
//				Tick window = deadline.minus(time);
//				return 1 - possibility.minus(time).doubleValue() / window.doubleValue();
			}

		}, 1000);
	}

	public LinkedList<Node> calculateSuprimePath() throws SchedulingException {
		return calculatePath(new Score() {
			@Override
			double score(Tick possibility, double load) {
				Tick window = deadline.minus(time);
				return (1 - possibility.minus(time).doubleValue() / window.doubleValue()) * load;
			}

		}, 100);
	}

	abstract class Score {
		abstract double score(Tick possibility, double load);
	}

	public LinkedList<Node> calculatePath(Score scoring, int threshold) throws SchedulingException {
		// memory optimalization
		double thres_value = 1.0;

		// initialize the paths :: <score, path> with a comparator that unsure the path with the best score are first in the list
		TreeSet<Pair<Double, LinkedList<Node>>> paths = new TreeSet<Pair<Double, LinkedList<Node>>>(new Comparator<Pair<Double, LinkedList<Node>>>() {
			@Override
			public int compare(Pair<Double, LinkedList<Node>> o1, Pair<Double, LinkedList<Node>> o2) {
				return o1.first.equals(o2.first) ? -1 : o2.first.compareTo(o1.first);
			}
		});

		// options for the first product step
		int firstIndex = productSteps.get(0).getIndex();
		Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> firstOptions = serviceOptions.get(firstIndex);
		for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> option : firstOptions.entrySet()) {
			AID equiplet = option.getKey();
			double load = equipletInfo.get(equiplet).first;

			Position nextPosition = equipletInfo.get(option.getKey()).second;
			Pair<Position, Position> route = new Pair<>(position, nextPosition);

			// check if the travel time from the route is know, and is not the the same as the previous
			if (!travelTimes.containsKey(route)) {
				throw new SchedulingException("route doesn't exists in travel time list: " + route);
			}

			Tick duration = option.getValue().first;
			Tick travel = travelTimes.get(route);
			Tick arrival = time.add(travel);
			Tick firstPossibility = deadline;

			// Time option is the time from (=option.first) until (=option.second) the equiplet is possible to perform the service
			for (Pair<Tick, Tick> timeOption : option.getValue().second) {
				// choose the best time to perform the product step
				// is the first available time earlier than first possibility and the product can arrive + duration is within the time window
				if (timeOption.first.lessThan(firstPossibility) && arrival.add(duration).lessThan(timeOption.second)) {

					// set the first possibility, the first is the time the equiplet is able to perform or when the product can arrive by the equiplet
					firstPossibility = timeOption.first.max(arrival);
				}
			}

			LinkedList<Node> path = new LinkedList<>();
			path.add(new Node(equiplet, firstPossibility, duration, firstIndex));

			double score = scoring.score(firstPossibility, load);

			paths.add(new Pair<Double, LinkedList<Node>>(score, path));
			// System.out.println(" added initial : " + new Pair<Double, LinkedList<Node>>(load, path));
		}

		while (!paths.isEmpty()) {
			if (MASConfiguration.VERBOSITY > 3) {
				// //System.out.println(" PATHS =" + Util.formatPairList(paths));
				System.out.println(" PATHS =" + paths.size());
			}

			// get the first possible path to explore with the highest score
			Pair<Double, LinkedList<Node>> first = paths.pollFirst();
			LinkedList<Node> path = first.second;
			double score = first.first;
			Node node = path.getLast();

			int index = node.getIndex() + 1;
			if (index < serviceOptions.size()) {
				// processing not finished
				Map<AID, Pair<Tick, List<Pair<Tick, Tick>>>> options = serviceOptions.get(index);
				for (Entry<AID, Pair<Tick, List<Pair<Tick, Tick>>>> option : options.entrySet()) {
					AID equiplet = option.getKey();
					double load = equipletInfo.get(equiplet).first;

					Position lastPosition = equipletInfo.get(node.getEquipletAID()).second;
					Position nextPosition = equipletInfo.get(option.getKey()).second;
					Pair<Position, Position> route = new Pair<>(lastPosition, nextPosition);

					// check if the travel time from the route is know, and is not the the same as the previous
					if (!travelTimes.containsKey(route) && !lastPosition.equals(nextPosition)) {
						throw new SchedulingException("route doesn't exists in travel time list: " + route);
					}

					Tick duration = option.getValue().first;
					Tick travel = lastPosition.equals(nextPosition) ? new Tick(0) : travelTimes.get(route);
					Tick arrival = node.getTime().add(node.getDuration()).add(travel);
					Tick firstPossibility = deadline;

					// Time option is the time from (=option.first) until (=option.second) the equiplet is possible to perform the service
					for (Pair<Tick, Tick> timeOption : option.getValue().second) {
						// choose the best time to perform the product step
						// is the first available time earlier than first possibility and the product can arrive + duration is within the time window
						if (timeOption.first.lessThan(firstPossibility) && arrival.add(duration).lessThan(timeOption.second)) {

							// set the first possibility, the first is the time the equiplet is able to perform or when the product can arrive by the equiplet
							firstPossibility = timeOption.first.max(arrival);
						}
					}

					Node nextNode = new Node(equiplet, firstPossibility, duration, index);
					if (MASConfiguration.VERBOSITY > 3) {
						System.out.println(" add to path [last in path=" + node + ", nextNode=" + nextNode + " + in path=" + path + "]");
					}

					if (firstPossibility.greaterOrEqualThan(deadline)) {
						throw new SchedulingException("failed to find path withing deadline=" + deadline);// + " best path so far=" + paths.first());
					}

					double newScore = score * scoring.score(firstPossibility, load);

					if (newScore > thres_value) {
						LinkedList<Node> newPath = new LinkedList<>(path);
						newPath.add(nextNode);
						paths.add(new Pair<Double, LinkedList<Node>>(newScore, newPath));

						if (paths.size() > threshold) {
							paths.remove(paths.last());
						}
					} else if (paths.size() < threshold) {
						thres_value = newScore;
						LinkedList<Node> newPath = new LinkedList<>(path);
						newPath.add(nextNode);
						paths.add(new Pair<Double, LinkedList<Node>>(newScore, newPath));
					}
				}
			} else {
				// found a good path
				return path;
			}
		}

		throw new SchedulingException("Scheduling: failed to find a path in paths: " + paths);
	}

	/**
	 * This scheduling algorithm is made by Leo van Moergestel and implemented
	 * by Laurens van den Brink, for further explanations and results see
	 * plublications.
	 * 
	 * The algorithm will make a matrix m x n with m equiplets and n product
	 * steps. The value of (r, c) with r < m and c < n will be 1 if the equiplet
	 * r can execute the product step c, otherwise 0. If the equiplet can
	 * execute multiple consecutive product steps the values will # of product
	 * steps - 1. The next steps is to multiply the values with the load of
	 * equiplet.
	 * 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user
	 *      requirements to product' - Leo van Moergestel section 3.2
	 * 
	 * @return the production steps
	 * @throws SchedulingException
	 */
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
				if (serviceOptions.get(productSteps.get(column).getIndex()).containsKey(equiplet)) {
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

		// apply load information to the scores
		for (int row = 0; row < equiplets.size(); row++) {
			AID equiplet = equiplets.get(row);
			double load = equipletInfo.get(equiplet).first;

			for (int column = 0; column < productSteps.size(); column++) {
				matrix[row][column] = matrix[row][column] * load;
			}
		}

		if (MASConfiguration.VERBOSITY > 3) {
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
				Pair<Tick, List<Pair<Tick, Tick>>> option = serviceOptions.get(productSteps.get(column).getIndex()).get(equiplet);
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

				if (MASConfiguration.VERBOSITY > 3) {
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
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more
	 * information see
	 * 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user
	 *      requirements to product' - Leo van Moergestel section 3.2
	 * 
	 * @param equiplets
	 *            Map < Equiplet, < List<Service able to perform>, load of
	 *            equiplet>
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
