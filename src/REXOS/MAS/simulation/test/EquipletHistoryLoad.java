package MAS.simulation.test;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import MAS.equiplet.Job;
import MAS.util.Tick;

public class EquipletHistoryLoad {

	@Test
	public void test1EmptySchedule() {
		System.out.println("\n testEmptySchedule");
		EquipletTest equiplet = new EquipletTest();
		double load = equiplet.loadHistory(new Tick(0), new Tick(1000));
		
		System.out.println("1: load must be 1 with an empty schedule: load=" + load);
		assertEquals("load must be 1 with an empty schedule", 1, load, 0);
	}

	@Test
	public void test2OneJobFullInside() {
		System.out.println("\n testOneJobFullInside");
		double window = 1000;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(10), new Tick(60)));
		double load = equiplet.loadHistory(new Tick(0), new Tick(window));
		
		System.out.println("2: load must be " + (1 - 50 / window) + "  with one job in schedule: load=" + load);
		assertEquals("load must be " + (1 - 50 / window) + "  with one job in schedule", (1 - 50 / window), load, 0);
	}

	@Test
	public void test3OneJobPartialInside() {
		System.out.println("\n testOneJobPartialInside");
		double window = 1000;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(window - 20), new Tick(window + 20)));
		double load = equiplet.loadHistory(new Tick(0), new Tick(window));

		System.out.println("3: load must be " + (1 - 20 / window) + "  with one job partial in window: load=" + load);
		assertEquals("load must be " + (1 - 20 / window) + "  with one job partial in window", (1 - 20 / window), load, 0);
	}

	@Test
	public void test4TwoJobPartialInside() {
		System.out.println("\n test executing");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(10), new Tick(60)));
		equiplet.executed(new Job(new Tick(window - 20), new Tick(window + 20)));
		double load = equiplet.loadHistory(new Tick(0), new Tick(window));

		System.out.println("4: load must be " + (1 - (50 + 20) / window) + "  with multiple jobs where one partial in window: load=" + load);
		assertEquals("load must be " + (1 - (50 + 20) / window) + "  with multiple jobs where one partial in window", (1 - (50 + 20) / window), load, 0);
	}

	@Test
	public void test5TwoJobOutside() {
		System.out.println("\n testTwoJobOutside");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(10), new Tick(60)));
		equiplet.executed(new Job(new Tick(window + 20), new Tick(window + 50)));
		double load = equiplet.loadHistory(new Tick(0), new Tick(window));

		System.out.println("5: load must be " + (1 - 50 / window) + "  with multiple jobs where one is outside the window: load=" + load);
		assertEquals("load must be " + (1 - 50 / window) + "  with multiple jobs where one is outside the window", (1 - 50 / window), load, 0);
	}

	@Test
	public void test6MultipleJobsInside() {
		System.out.println("\n testMultipleJobsInside");
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		int sumDuration = 0;
		for (int i = 50; i < 600; i += 80) {
			if (i > 100)
				sumDuration += 30;
			equiplet.executed(new Job(new Tick(i), new Tick(i + 30)));
		}

		double load = equiplet.loadHistory(new Tick(100), new Tick(window));
		System.out.println("6: load must be " + (1 - sumDuration / window) + "  with an empty schedule: load=" + load);
		assertEquals("load must be " + (1 - sumDuration / window) + "  with an empty schedule", 1 - sumDuration / window, load, 0);
	}

	@Test
	public void test7FullSchedule() {
		System.out.println("\n testFullSchedule");
		double window = 500d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(0), new Tick(50)));
		equiplet.executed(new Job(new Tick(50), new Tick(100)));
		equiplet.executed(new Job(new Tick(100), new Tick(150)));
		equiplet.executed(new Job(new Tick(150), new Tick(200)));
		equiplet.executed(new Job(new Tick(200), new Tick(250)));
		equiplet.executed(new Job(new Tick(250), new Tick(300)));
		equiplet.executed(new Job(new Tick(300), new Tick(350)));
		equiplet.executed(new Job(new Tick(350), new Tick(400)));
		equiplet.executed(new Job(new Tick(400), new Tick(450)));
		equiplet.executed(new Job(new Tick(450), new Tick(500)));
		equiplet.executed(new Job(new Tick(500), new Tick(550)));

		double load = equiplet.loadHistory(new Tick(40), new Tick(window));
		System.out.println("7: load must be " + (1 - window / window) + "  with an full schedule " + load);
		assertEquals("load must be " + (1 - window / window) + "  with an empty schedule", (1 - (window / window)), load, 0);
	}

	@Test
	public void testWindows8EqualToSchedule() {
		System.out.println("\n testWindowsEqualToSchedule");
		System.out.println("double problem: (1 - 0.1 - 0.1 - 0.1 - 0.1) = " + (1 - 0.1 - 0.1 - 0.1 - 0.1) + " != 0.6");
		double window = 100d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(16), new Tick(66)));
		equiplet.executed(new Job(new Tick(66), new Tick(116)));
		equiplet.executed(new Job(new Tick(116), new Tick(166)));
		equiplet.executed(new Job(new Tick(166), new Tick(216)));

		double load = equiplet.loadHistory(new Tick(28.269870205410786), new Tick(window));
		System.out.println("8: load must be " + (1 - window / window) + "  with an full schedule " + load);
		assertEquals("load must be " + (1 - window / window) + "  with an empty schedule", (1 - (window / window)), load, 0);
	}

	@Test
	public void test9FullScheduleExecuting() {
		System.out.println("\n testFullSchedule with executing");
		double window = 500d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(0), new Tick(50)));
		equiplet.executed(new Job(new Tick(50), new Tick(100)));
		equiplet.executed(new Job(new Tick(100), new Tick(150)));
		equiplet.executed(new Job(new Tick(150), new Tick(200)));
		equiplet.executed(new Job(new Tick(200), new Tick(250)));
		equiplet.executed(new Job(new Tick(250), new Tick(300)));
		equiplet.executed(new Job(new Tick(300), new Tick(350)));
		equiplet.executed(new Job(new Tick(350), new Tick(400)));
		equiplet.executed(new Job(new Tick(400), new Tick(450)));
		equiplet.executed(new Job(new Tick(450), new Tick(500)));
		equiplet.execute(new Job(new Tick(500), new Tick(550)));

		double load = equiplet.loadHistory(new Tick(20), new Tick(window));
		System.out.println("7: load must be " + (1 - window / window) + "  with an full schedule " + load);
		assertEquals("load must be " + (1 - window / window) + "  with an empty schedule", (1 - (window / window)), load, 0);
	}
	
	@Test
	public void test10Aaarrrggg() {
		double window = 1000d;

		EquipletTest equiplet = new EquipletTest();
		equiplet.executed(new Job(new Tick(46.00), new Tick(166.00)));
		equiplet.executed(new Job(new Tick(166.00), new Tick(286.00)));
		equiplet.executed(new Job(new Tick(313.00), new Tick(433.00)));
		equiplet.executed(new Job(new Tick(456.00), new Tick(576.00)));
		equiplet.executed(new Job(new Tick(576.00), new Tick(696.00)));
		equiplet.executed(new Job(new Tick(719.00), new Tick(839.00)));
		equiplet.executed(new Job(new Tick(839.00), new Tick(959.00)));
		equiplet.executed(new Job(new Tick(1016.00), new Tick(1136.00)));
		
		equiplet.execute(new Job(new Tick(1180), new Tick(1300)));
		
		double load = equiplet.loadHistory(new Tick(240), new Tick(window));
		System.out.println("10: load must be " + (1 - 826 / window) + "  with an full schedule " + load);
		assertEquals("load must be " + (1 - 826 / window) + "  with an empty schedule", (1 - (826 / window)), load, 0);
	}
}
