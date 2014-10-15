package MAS.simulation.test;

import jade.core.AID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

import org.jfree.data.gantt.Task;
import org.jfree.data.gantt.TaskSeries;
import org.jfree.data.time.SimpleTimePeriod;
import org.jfree.ui.RefineryUtilities;

import MAS.simulation.graphics.GanttChart;
import MAS.simulation.mas.equiplet.Job;
import MAS.simulation.mas.product.Graph;
import MAS.simulation.mas.product.Node;
import MAS.simulation.mas.product.Product;
import MAS.simulation.mas.product.ProductStep;
import MAS.simulation.mas.product.ProductionStep;
import MAS.simulation.simulation.Grid;
import MAS.simulation.util.Pair;
import MAS.simulation.util.Position;
import MAS.simulation.util.Tick;

class Test {
	public class Entry implements Comparable<Entry> {
		Integer start;
		Integer end;

		public Entry(Integer s, Integer e) {
			start = s;
			end = e;
		}

		@Override
		public boolean equals(Object obj) {
			if (obj instanceof Entry) {
				return compareTo((Entry) obj) == 0;
			}
			return false;
		}

		@Override
		public int compareTo(Entry o) {
			if (o.end != null // other ends before or when this starts
					&& (o.end.equals(start) || o.end < start)) {
				return 1;
			}
			if (end != null // other starts after or when this ends
					&& (o.start.equals(end) || o.start > end)) {
				return -1;
			}
			return 0;
		}

		public String toString() {
			return "[" + start + "-" + end + "]";
		}
	}

	public static void main(String[] args) {
		new Test();
	}

	public Test() {
		removeOutTreeSet();
	}

	private void removeOutTreeSet() {
		TreeSet<Job> map = new TreeSet<>();
		
		map.add(new Job(0, new AID("P0"), "service", new HashMap<String, Object>(), new Tick(1), new Tick(5),new Tick(20)));
		map.add(new Job(1, new AID("P1"), "service", new HashMap<String, Object>(), new Tick(5), new Tick(10),new Tick(20)));
		map.add(new Job(2, new AID("P2"), "service", new HashMap<String, Object>(), new Tick(15), new Tick(20),new Tick(20)));

		Job nr2 = null;
		int i = 0;
		for (Job entry : map) {
			if (i == 1) {
				nr2 = entry;
				break;
			}
			i++;
		}
		System.out.println("map: " + map);

		Job first = map.first();
		first.updateDueTime(nr2.getStartTime());

		map.remove(nr2);
		System.out.println("nr2: " + nr2);
		System.out.println("map: " + map);
	}

	@SuppressWarnings("unused")
	private void scheduleTest() {
		TreeSet<Entry> map = new TreeSet<>();
		map.add(new Entry(1, 5));
		map.add(new Entry(5, 10));
		map.add(new Entry(15, 20));

		SortedSet<Entry> subSet = map.subSet(new Entry(1, 10), true, new Entry(15, 15), true);
		System.out.println("map: " + map);
		System.out.println("sub set: " + subSet);
	}

	@SuppressWarnings("unused")
	private void routeTest() {

		Set<Pair<Position, Position>> routes = new HashSet<>();
		Position pos1 = new Position(0, 0);
		Position pos2 = new Position(2, 0);
		Position pos3 = new Position(2, 0);
		Position pos4 = new Position(0, 0);

		Pair<Position, Position> route1 = new Pair<Position, Position>(pos1, pos2);
		Pair<Position, Position> route2 = new Pair<Position, Position>(pos4, pos3);

		routes.add(route1);
		routes.add(route2);

		System.out.printf("pos1 %s == pos4 %s = %b\n\n", pos1, pos4, (pos1.equals(pos4)));
		System.out.printf("route1 %s == route1 %s = %b\n\n", route1, route1, (route1.equals(route1)));
		System.out.println("routes: " + routes);
		// productScheduling();
	}

	public void testGraph() {
		Graph<TNode> graph = new Graph<TNode>();

		TNode source = new TNode("source");
		TNode sink = new TNode("sink");

		// rij 1
		TNode n1 = new TNode("node 1");
		TNode n2 = new TNode("node 2");
		TNode n3 = new TNode("node 3");
		// rij 2
		TNode n4 = new TNode("node 4");
		TNode n5 = new TNode("node 5");
		// rij 3
		TNode n6 = new TNode("node 6");
		TNode n7 = new TNode("node 7");
		TNode n8 = new TNode("node 8");

		graph.add(source, n1, 0);
		graph.add(source, n2, 0);
		graph.add(source, n3, 0);

		graph.add(n1, n4, 10);
		graph.add(n1, n5, 12);

		graph.add(n2, n4, 5);
		graph.add(n3, n5, 5);

		graph.add(n4, n6, 10);
		graph.add(n5, n7, 9);
		graph.add(n5, n8, 8);

		graph.add(n6, sink, 0);
		graph.add(n7, sink, 0);
		graph.add(n8, sink, 0);

		LinkedList<TNode> path = graph.optimumPath(source, sink);
		System.out.println("graph: " + graph);
		System.out.println("path: " + path);
	}

	@SuppressWarnings("unused")
	public static void productScheduling() {
		System.out.println("Start product scheduling test...");

		System.out.println("TEst NODE + " + new Node(new Tick(4)));
		LinkedList<ProductStep> productSteps = new LinkedList<ProductStep>();
		// productSteps.add(new ProductStep(1, "screw", new HashMap<String, Object>()));
		// productSteps.add(new ProductStep(2, "glue", new HashMap<String, Object>()));

		/*
		 * Product p0 = new Product("P0", 0, productSteps, new Position(0,0));
		 * System.out.println("P1: " + p0 + "\n");
		 * 
		 * Product p1 = new Product("P1", 0, productSteps, new Position(0,0));
		 * System.out.println("P1: " + p1 + "\n");
		 * 
		 * Product p2 = new Product("P2", 10, productSteps, new Position(0,0));
		 * System.out.println("P2: " + p2 + "\n");
		 * 
		 * Product p3 = new Product("P3", 20, productSteps, new Position(0,0));
		 * System.out.println("P3: " + p3 + "\n");
		 * 
		 * Product p4 = new Product("P4", 30, productSteps, new Position(0,0));
		 * System.out.println("P4: " + p4 + "\n");
		 * 
		 * Product p5 = new Product("P5", 40, productSteps, new Position(0,0));
		 * System.out.println("P5: " + p5 + "\n");
		 * 
		 * 
		 * ArrayList<Product> agents = new ArrayList<>();
		 * agents.add(p1);
		 * agents.add(p2);
		 * agents.add(p3);
		 * agents.add(p4);
		 * agents.add(p5);
		 * output(agents);
		 */
	}

	/**
	 * This test whether the scheduling of a product fails when there are no
	 * capable equiplets
	 */
	@SuppressWarnings("unused")
	public void productSchedulingFailed() {
		System.out.println("Start product scheduling test...");

		LinkedList<ProductStep> productSteps = new LinkedList<ProductStep>();
		// productSteps.add(new ProductStep(1, "Fails", new HashMap<String, Object>()));
		// Product p1 = new Product("PFAILED", 0, productSteps, new Position(0,0));

		// System.out.println("P1: " + p1);

		Grid grid = Grid.getInstance();
		System.out.println("grid: " + grid);
	}

	/**
	 * give the output of the product agents
	 * 
	 * @param agents
	 */
	@SuppressWarnings("unused")
	private static void output(ArrayList<Product> agents) {
		ArrayList<TaskSeries> tasks = new ArrayList<>();

		for (Product agent : agents) {
			LinkedList<ProductionStep> path = agent.getProductionPath();
			TaskSeries serie = new TaskSeries(agent.getProductName());
			for (ProductionStep step : path) {
				serie.add(new Task(step.getEquipletName(), new SimpleTimePeriod((long) step.getStart().doubleValue(), (long) (step.getStart().doubleValue() + step.getDuration().doubleValue()))));
			}
			tasks.add(serie);
		}

		final GanttChart graph = new GanttChart("Schedule Chart", tasks);
		graph.pack();
		RefineryUtilities.centerFrameOnScreen(graph);
		graph.setVisible(true);
	}

	/**
	 * Test node for the graph
	 */
	static class TNode extends Node {
		String name;

		public TNode(String name) {
			this.name = name;
		}

		@Override
		public String toString() {
			return name;
		}
	}
}
