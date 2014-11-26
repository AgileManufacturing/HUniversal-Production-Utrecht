package MAS.simulation.test;

import static org.junit.Assert.assertArrayEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import MAS.equiplet.Job;
import MAS.util.Pair;
import MAS.util.Tick;

import static org.junit.Assert.*;




public class EquipletAvailability {

	@Test
	public void showcase(){
		boolean x = true;
		assertFalse("Showcase wants a false but got a "+x+"\n" ,x);
	}

	@Test
	public void test1() {
		Tick start = new Tick(10);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n1: Test without jobs in schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");

		EquipletTest equiplet = new EquipletTest();
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(start, deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test2() {
		Tick start = new Tick(10);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n2: Test with 1 job inside the begin of the schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");

		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(5), new Tick(35)));

		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(new Tick(35d), deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("Availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test3() {
		Tick start = new Tick(10);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n3: Test with 2 job inside the schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(5), new Tick(50)));
		equiplet.schedule(new Job(new Tick(60), new Tick(100)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(new Tick(100d), deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test4() {
		Tick start = new Tick(10);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n4: Test with 1 job fully inside in schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(90), new Tick(140)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(start, new Tick(90d)));
		result.add(new Pair<>(new Tick(140d), deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test5() {
		Tick start = new Tick(10);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n5: Test with 1 outside job in schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(1050), new Tick(1100)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(start, deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test6() {
		Tick start = new Tick(10);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n6: Test with 1 executing and 1 job at begin schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.execute(new Job(new Tick(0), new Tick(45)));
		equiplet.schedule(new Job(new Tick(45), new Tick(90)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(new Tick(90d), deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test7() {
		Tick start = new Tick(15);
		Tick duration = new Tick(50);
		Tick deadline = new Tick(1000);

		//System.out.println("\n7: Test with 1 executing [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.execute(new Job(new Tick(10), new Tick(69)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(new Tick(69d), deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test8() {
		Tick start = new Tick(0);
		Tick duration = new Tick(5);
		Tick deadline = new Tick(75);

		//System.out.println("\n8: Test with multiple job in schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(5), new Tick(10)));
		equiplet.schedule(new Job(new Tick(20), new Tick(25)));
		equiplet.schedule(new Job(new Tick(28), new Tick(32)));
		equiplet.schedule(new Job(new Tick(40), new Tick(45)));
		equiplet.schedule(new Job(new Tick(48), new Tick(53)));
		equiplet.schedule(new Job(new Tick(80), new Tick(85)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();
		result.add(new Pair<>(new Tick(10d), new Tick(20d)));
		result.add(new Pair<>(new Tick(32d), new Tick(40d)));
		result.add(new Pair<>(new Tick(53d), deadline));

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}

	@Test
	public void test9() {
		Tick start = new Tick(10);
		Tick duration = new Tick(5);
		Tick deadline = new Tick(500);

		//System.out.println("\n9: Test with a full schedule [start=" + start + ", duration=" + duration + ", deadline=" + deadline + "]");
		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(0), new Tick(50)));
		equiplet.schedule(new Job(new Tick(50), new Tick(100)));
		equiplet.schedule(new Job(new Tick(100), new Tick(150)));
		equiplet.schedule(new Job(new Tick(150), new Tick(200)));
		equiplet.schedule(new Job(new Tick(200), new Tick(250)));
		equiplet.schedule(new Job(new Tick(250), new Tick(300)));
		equiplet.schedule(new Job(new Tick(300), new Tick(350)));
		equiplet.schedule(new Job(new Tick(350), new Tick(400)));
		equiplet.schedule(new Job(new Tick(400), new Tick(450)));
		equiplet.schedule(new Job(new Tick(450), new Tick(500)));
		equiplet.schedule(new Job(new Tick(500), new Tick(550)));
		List<Pair<Tick, Tick>> available = equiplet.available(start, duration, deadline);

		List<Pair<Tick, Tick>> result = new ArrayList<>();

		//System.out.println("Available: " + result + " == " + available);
		assertArrayEquals("availability must be " + result + "  with one job in schedule", result.toArray(), available.toArray());
	}
}
