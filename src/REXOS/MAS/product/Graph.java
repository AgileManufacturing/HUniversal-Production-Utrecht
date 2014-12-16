package MAS.product;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Stack;

import MAS.util.Pair;
import MAS.util.MasConfiguration;

public class Graph<V extends Node> {

	private HashMap<V, HashMap<V, Double>> graph;
	private int arcs;

	public Graph() {
		this.graph = new HashMap<V, HashMap<V, Double>>();
		this.arcs = 0;
	}

	/**
	 * Add a vertex to the graph. Nothing happens if vertex is already in graph.
	 */
	public void add(V vertex) {
		if (!this.contains(vertex)) {
			graph.put(vertex, new HashMap<V, Double>());
		}
	}

	/**
	 * Add an edge to the graph; if either vertex does not exist, it's added.
	 * This implementation allows the creation of multi-edges and self-loops.
	 */
	public boolean add(V from, V to, double cost) {
		boolean added = false;

		if (!this.contains(to)) {
			graph.put(to, new HashMap<V, Double>());
			added = true;
		}
		if (!this.contains(from)) {
			HashMap<V, Double> neighbors = new HashMap<V, Double>();
			neighbors.put(to, cost);
			graph.put(from, neighbors);
		} else {
			graph.get(from).put(to, cost);
		}
		System.err.println("Graph [node=" + graph.size() + ", arcs=" + ++arcs + "]");

		return added;
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
		while (!paths.isEmpty()) {
			Pair<LinkedList<V>, Double> p = paths.pop();
			LinkedList<V> subPath = p.first;
			V node = subPath.getLast();

			// Check whether if the node is already processed to avoid infinite looping
			// TODO think about this, this should be there otherwise it doesn't work
			// if (!processed.contains(node)) {
			HashMap<V, Double> neighbors = graph.get(node);
			processed.add(node);

			if (MasConfiguration.VERBOSITY > 3) {
				System.out.println("process node: " + node + " with neighbors " + neighbors);
			}

			// Add each neighbor to the path with the cost
			for (Entry<V, Double> neighbor : neighbors.entrySet()) {
				LinkedList<V> newSubPath = new LinkedList<V>(subPath);
				newSubPath.add(neighbor.getKey());
				double cost = p.second + neighbor.getValue();
				paths.add(new Pair<LinkedList<V>, Double>(newSubPath, cost));

				if (MasConfiguration.VERBOSITY > 3) {
					System.out.println("newPath:  [neighbor=" + neighbor.getKey() + ", to=" + to + ", cost=" + cost + ", score " + score + ", path=" + newSubPath + "]");
				}

				// Check whether the path is the best and should be returned
				if (neighbor.getKey().equals(to) && cost > score) {
					if (MasConfiguration.VERBOSITY > 3) {
						System.out.println("WIN path:  [to=" + to + ", cost=" + cost + ", path " + newSubPath + ", score " + score + "]");
					}
					score = cost;
					path = newSubPath;
				}
				// }
			}
		}

		if (MasConfiguration.VERBOSITY > 3) {
			System.out.println("Optimum path score=" + score + " of path=" + path);
		}
		return path;
	}

	/**
	 * String representation of graph.
	 */
	@Override
	public String toString() {
		StringBuffer s = new StringBuffer();

		List<V> keys = new ArrayList<V>(graph.keySet());
		Collections.sort(keys, new Comparator<V>() {
			public int compare(V o1, V o2) {
				return o1.compareTo(o2);
			}
		});

		// for (V v : graph.keySet()) {
		for (V v : keys) {
			s.append("\n    " + v + " -> " + graph.get(v));
		}
		return s.toString();
	}

	public String prettyPrint(V from) {
		LinkedList<List<V>> g = new LinkedList<>();

		ArrayList<V> nodes = new ArrayList<V>();
		for (Entry<V, HashMap<V, Double>> entry : graph.entrySet()) {
			if (!nodes.contains(entry.getKey())) {
				nodes.add(entry.getKey());
			}
		}

		ArrayList<V> level = new ArrayList<V>();
		ArrayList<V> nextLevel = new ArrayList<V>();
		level.add(from);

		int largestLevel = 0;

		while (!level.isEmpty()) {
			for (V node : level) {
				HashMap<V, Double> neighbors = graph.get(node);
				for (Entry<V, Double> neighbor : neighbors.entrySet()) {
					if (!nextLevel.contains(neighbor.getKey())) {
						nextLevel.add(neighbor.getKey());
					}
				}
			}
			largestLevel = Math.max(largestLevel, nextLevel.size());

			g.add(level);
			level = new ArrayList<>(nextLevel);
			nextLevel = new ArrayList<>();
		}

		StringBuilder s = new StringBuilder();
		Iterator<List<V>> iterator = g.iterator();
		while (iterator.hasNext()) {
			List<V> lvl = iterator.next();

			String space = " ";
			int diff = (largestLevel - lvl.size());
			for (int i = 0; i <= diff; i++) {
				space += " ";
			}

			s.append("\n");
			for (V v : lvl) {
				nodes.remove(v);
				// s.append(space + (v.getEquipletAID() != null ? v.getEquipletAID().getLocalName() : v.getEquiplet() != null ? v.getEquiplet() : v.getTime().greaterThan(-1) ?
				// "source" : "sink") + "," + space);
				s.append(space + v + "," + space);
			}
			s.append("\n");
		}

		s.append("\n");
		s.append("remaining: " + nodes);
		s.append("\n");

		return s.toString();
	}
}
