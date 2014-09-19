package MAS.simulation.mas.product;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Stack;

import MAS.simulation.util.Pair;
import MAS.simulation.util.Settings;

public class Graph<V> {

	private Map<V, HashMap<V, Double>> graph;

	public Graph() {
		this.graph = new HashMap<V, HashMap<V, Double>>();
	}

	/**
	 * Add a vertex to the graph. Nothing happens if vertex is already in graph.
	 */
	public void add(V vertex) {
		if (!graph.containsKey(vertex)) {
			graph.put(vertex, new HashMap<V, Double>());
		}
	}

	/**
	 * Add an edge to the graph; if either vertex does not exist, it's added.
	 * This implementation allows the creation of multi-edges and self-loops.
	 */
	public void add(V from, V to, double cost) {
		if (!graph.containsKey(to)) {
			graph.put(to, new HashMap<V, Double>());
		}
		if (!graph.containsKey(from)) {
			HashMap<V, Double> neighbors = new HashMap<V, Double>();
			neighbors.put(to, cost);
			graph.put(from, neighbors);
		} else {
			graph.get(from).put(to, cost);
		}
	}

	/**
	 * True iff graph contains vertex.
	 */
	public boolean contains(V vertex) {
		return graph.containsKey(vertex);
	}

	/**
	 * Remove an edge from the graph. Nothing happens if no such edge.
	 * 
	 * @throws IllegalArgumentException
	 *             if either vertex doesn't exist.
	 */
	public void remove(V from, V to) {
		if (!(this.contains(from) && this.contains(to))) {
			throw new IllegalArgumentException("Nonexistent vertex");
		}
		graph.get(from).remove(to);
	}

	/**
	 * Calculate the path with the lowest cost
	 */
	public LinkedList<V> optimumPath(V from, V to) {
		Stack<Pair<LinkedList<V>, Double>> paths = new Stack<>();
		LinkedList<V> start = new LinkedList<V>();
		start.add(from);
		paths.add(new Pair<LinkedList<V>, Double>(start, 0.0));

		// initialize
		LinkedList<V> path = new LinkedList<>();
		double score = Double.MIN_VALUE;
		ArrayList<V> processed = new ArrayList<V>();

		// System.out.println("Compute the path...");

		// Compute the path, for each path make add a new path with the neighbors
		// for (Entry<LinkedList<V>, Integer> p : paths.entrySet()) {
		while (!paths.isEmpty()) {
			Pair<LinkedList<V>, Double> p = paths.pop();
			LinkedList<V> subPath = p.first;
			V node = subPath.getLast();

			// Check whether if the node is already processed to avoid infinite looping 
			if (!processed.contains(node)) {
				HashMap<V, Double> neighbors = graph.get(node);
				processed.add(node);

				if (Settings.VERBOSITY > 3) {
					System.out.println("process node: " + node + " with neighbors " + neighbors);
				}
				
				// Add each neighbor to the path with the cost
				for (Entry<V, Double> neighbor : neighbors.entrySet()) {
					LinkedList<V> newSubPath = new LinkedList<V>(subPath);
					newSubPath.add(neighbor.getKey());
					double cost = p.second + neighbor.getValue();
					paths.add(new Pair<LinkedList<V>, Double>(newSubPath, cost));

					if (Settings.VERBOSITY > 3) {
						System.out.println("newPath:  [neighbor=" + neighbor.getKey() + ", to=" + to + ", cost=" + cost + ", score " + score + ", path=" + newSubPath + "]");
					}

					// Check whether the path is the best and should be returned 
					if (neighbor.getKey().equals(to) && cost > score) {
						if (Settings.VERBOSITY > 3) {
							System.out.println("WIN path:  [to=" + to + ", cost=" + cost + ", path " + newSubPath + ", score " + score + "]");
						}
						score = cost;
						path = newSubPath;
					}
				}
			}
		}
		return path;
	}

	/**
	 * String representation of graph.
	 */
	@Override
	public String toString() {
		StringBuffer s = new StringBuffer();
		for (V v : graph.keySet()) {
			s.append("\n    " + v + " -> " + graph.get(v));
		}
		return s.toString();
	}
}
