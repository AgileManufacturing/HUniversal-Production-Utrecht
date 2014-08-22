package simulation.test;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;

import simulation.mas.equiplet.Equiplet;
import simulation.mas.equiplet.EquipletAgent;
import simulation.mas.equiplet.Job;
import simulation.util.Pair;

public class EquipletAvailability extends EquipletAgent {

	public static void main(String[] args) {

		System.out.println("\n1: Test without jobs in schedule");

		EquipletAvailability equiplet1 = new EquipletAvailability();
		System.out.println("Equiplet: " + equiplet1);

		List<Pair<Double, Double>> available1 = equiplet1.available(2, 5, 75);
		System.out.println("Available: " + available1);

		
		//
		System.out.println("\n2: Test with one job in schedule");

		EquipletAvailability equiplet2 = new EquipletAvailability();
		equiplet2.schedule(new Job(5, 10));
		System.out.println("Equiplet: " + equiplet2);

		List<Pair<Double, Double>> available2 = equiplet2.available(2, 5, 75);
		System.out.println("Available: " + available2);
		

		//
		System.out.println("\n3: Test with one job in schedule");

		EquipletAvailability equiplet3 = new EquipletAvailability();
		equiplet3.schedule(new Job(5, 10));
		equiplet3.schedule(new Job(28, 32));
		System.out.println("Equiplet: " + equiplet3);

		List<Pair<Double, Double>> available3 = equiplet3.available(2, 5, 75);
		System.out.println("Available: " + available3);
		
		
		//
		System.out.println("\n4: Test with one job in schedule");

		EquipletAvailability equiplet4 = new EquipletAvailability();
		equiplet4.schedule(new Job(50, 55));
		System.out.println("Equiplet: " + equiplet4);

		List<Pair<Double, Double>> available4 = equiplet4.available(2, 5, 75);
		System.out.println("Available: " + available4);		
		
		
		//
		System.out.println("\n5: Test with one job in schedule");

		EquipletAvailability equiplet5 = new EquipletAvailability();
		equiplet5.schedule(new Job(80, 85));
		System.out.println("Equiplet: " + equiplet5);

		List<Pair<Double, Double>> available5 = equiplet5.available(2, 5, 75);
		System.out.println("Available: " + available5);
		
		
		//
		System.out.println("\n6: Test with one job in schedule");

		EquipletAvailability equiplet6 = new EquipletAvailability();
		equiplet6.execute(new Job(205, 215));
		equiplet6.schedule(new Job(215, 225));
		System.out.println("Equiplet: " + equiplet6);
		
		List<Pair<Double, Double>> available6 = equiplet6.available(210, 10, 10210);
		System.out.println("Available: " + available6);

		
		//
		System.out.println("\n7: Test with multiple job in schedule");
		EquipletAvailability equiplet = new EquipletAvailability();
		equiplet.schedule(new Job(5, 10));
		equiplet.schedule(new Job(20, 25));
		equiplet.schedule(new Job(28, 32));
		equiplet.schedule(new Job(40, 45));
		equiplet.schedule(new Job(48, 53));
		equiplet.schedule(new Job(80, 85));

		System.out.println("Equiplet: " + equiplet);

		List<Pair<Double, Double>> available = equiplet.available(2, 5, 75);
		System.out.println("Available: " + available);

	}

	public EquipletAvailability() {
		schedule = new TreeSet<>();
	}

	protected void execute(Job job) {
		executing = job;
	}
	
	private void schedule(Job job) {
		schedule.add(job);
	}

	public String toString() {
		StringBuilder builder = new StringBuilder();
		builder.append("EQ schedule:");
		if (isExecuting()) {
			builder.append("\nExe\t" + executing);
		}
		for (Job j : schedule) {
			builder.append("\n\t" + j);
		}
		return builder.toString();
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/*
	protected List<Pair<Double, Double>> available(double time, double duration, double deadline) {
		List<Pair<Double, Double>> available = new ArrayList<Pair<Double, Double>>();

		double start = time;
		if (isExecuting()) {
			start = Math.max(start, executing.getDue());
		}

		
		if (schedule.isEmpty()) {
			available.add(new Pair<Double, Double>(start, deadline));
		} else {
			Iterator<Job> it = schedule.iterator();
			while (it.hasNext()) {
				Job job = it.next();

				// System.out.println("(J.start = " + job.getStartTime() + " > start = " + start + " && " + (job.getStartTime() - start) + " > " + duration + " )");

				if (job.getStartTime() > start) {

					if (job.getStartTime() - start > duration) {

						if (job.getStartTime() < deadline) {
							available.add(new Pair<Double, Double>(start, job.getStartTime()));
							start = job.getDue();
						} else {
							available.add(new Pair<Double, Double>(start, deadline));
							break;
						}
					} else {
						start = job.getDue();
					}
				}
				
				if (!it.hasNext()) {
					available.add(new Pair<Double, Double>(start, deadline));
				}
			}
		}
		return available;

	}
	*/

}
