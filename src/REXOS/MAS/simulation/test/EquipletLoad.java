package MAS.simulation.test;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import MAS.equiplet.Job;
import MAS.util.Tick;

public class EquipletLoad {

	@Test
	public void testEmptySchedule() {
		System.out.println("\n testEmptySchedule");
		EquipletTest equiplet = new EquipletTest();
		double load = equiplet.load(new Tick(0), new Tick(1000));
		assertEquals("load must be 1 with an empty schedule", 1, load, 0);
	}

	@Test
	public void testOneJobFullInside() {
		System.out.println("\n testOneJobFullInside");
		double window = 1000;

		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(10), new Tick(60)));
		double load = equiplet.load(new Tick(0), new Tick(window));
		assertEquals("load must be " + (1 - 50 / window) + "  with one job in schedule", (1 - 50 / window), load, 0);
	}

	@Test
	public void testOneJobPartialInside() {
		System.out.println("\n testOneJobPartialInside");
		double window = 1000;

		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(window - 20), new Tick(window + 20)));
		double load = equiplet.load(new Tick(0), new Tick(window));
		assertEquals("load must be " + (1 - 20 / window) + "  with one job partial in window", (1 - 20 / window), load, 0);
	}

	@Test
	public void testTwoJobPartialInside() {
		System.out.println("\n test executing");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(10), new Tick(60)));
		equiplet.schedule(new Job(new Tick(window - 20), new Tick(window + 20)));
		double load = equiplet.load(new Tick(0), new Tick(window));
		assertEquals("load must be " + (1 - (50 + 20) / window) + "  with multiple jobs where one partial in window", (1 - (50 + 20) / window), load, 0);
	}

	@Test
	public void testTwoJobOutside() {
		System.out.println("\n testTwoJobOutside");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(10), new Tick(60)));
		equiplet.schedule(new Job(new Tick(window + 20), new Tick(window + 50)));
		double load = equiplet.load(new Tick(0), new Tick(window));
		assertEquals("load must be " + (1 - 50 / window) + "  with multiple jobs where one is outside the window", (1 - 50 / window), load, 0);
	}

	@Test
	public void testMultipleJobsInside() {
		System.out.println("\n testMultipleJobsInside");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		int sumDuration = 0;
		for (int i = 50; i < 600; i += 80) {
			if (i > 100)
				sumDuration += 30;
			equiplet.schedule(new Job(new Tick(i), new Tick(i + 30)));
		}

		double load = equiplet.load(new Tick(100), new Tick(window));
		assertEquals("load must be " + (1 - sumDuration / window) + "  with an empty schedule", 1 - sumDuration / window, load, 0);
	}

	@Test
	public void testExecuting() {
		System.out.println("\n testExecuting");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.execute(new Job(new Tick(50), new Tick(100)));

		double load = equiplet.load(new Tick(60), new Tick(window));
		System.out.println("load must be " + (1 - 0 / window) + "  with an empty schedule " + load);
		assertEquals("load must be " + (1 - 0 / window) + "  with an empty schedule", (1 - (0 / window)), load, 0);
	}

	@Test
	public void testFullSchedule() {
		System.out.println("\n testFullSchedule");
		double window = 500d;

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

		double load = equiplet.load(new Tick(40), new Tick(window));
		System.out.println("load must be " + (1 - window / window) + "  with an full schedule " + load);
		assertEquals("load must be " + (1 - window / window) + "  with an empty schedule", (1 - (window / window)), load, 0);
	}

	@Test
	public void testWindowsEqualToSchedule() {
		System.out.println("\n testWindowsEqualToSchedule");
		System.out.println("double problem: (1 - 0.1 - 0.1 - 0.1 - 0.1) = " + (1 - 0.1 - 0.1 - 0.1 - 0.1) + " != 0.6");
		double window = 100d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.schedule(new Job(new Tick(16), new Tick(66)));
		equiplet.schedule(new Job(new Tick(66), new Tick(116)));
		equiplet.schedule(new Job(new Tick(116), new Tick(166)));
		equiplet.schedule(new Job(new Tick(166), new Tick(216)));

		double load = equiplet.load(new Tick(28.269870205410786), new Tick(window));
		System.out.println("load must be " + (1 - window / window) + "  with an full schedule " + load);
		assertEquals("load must be " + (1 - window / window) + "  with an empty schedule", (1 - (window / window)), load, 0);
	}
}
