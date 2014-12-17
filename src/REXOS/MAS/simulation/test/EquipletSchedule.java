package MAS.simulation.test;

import static org.junit.Assert.fail;

import org.junit.Test;

import MAS.equiplet.Job;
import MAS.util.Tick;

public class EquipletSchedule {

	@Test
	public void test1() {
		System.out.println();
		System.out.println("test 1 scheduling of job exactly between two other jobs");

		EquipletTest equiplet = new EquipletTest();
		// job in schedule
		equiplet.schedule(new Job(new Tick(0), new Tick(50)));
		equiplet.schedule(new Job(new Tick(100), new Tick(150)));

		try {
			// possible failure
			equiplet.schedule(new Tick(50), new Tick(50));
			System.out.println("equiplet: " + equiplet);
		} catch (IllegalArgumentException e) {
			System.out.println("equiplet: " + equiplet);
			System.err.println("overlap in schedule incorrectly detected: " + e.getMessage());
			e.printStackTrace();
			fail("overlap in schedule incorrectly detected");
		}
	}

	@Test
	public void test2() {
		System.out.println();
		System.out.println("test 2 scheduling of job where overlap is the job");

		EquipletTest equiplet = new EquipletTest();
		// job in schedule
		equiplet.schedule(new Job(new Tick(0), new Tick(50)));
		equiplet.schedule(new Job(new Tick(100), new Tick(150)));

		try {
			// possible failure
			equiplet.schedule(new Tick(10), new Tick(50));
			System.out.println("equiplet: " + equiplet);
			fail("no overlap in schedule detected");
		} catch (IllegalArgumentException e) {
			System.out.println("equiplet: " + equiplet);
			System.err.println("overlap in schedule correctly detected: " + e.getMessage());
			e.printStackTrace();
		}
	}

	@Test
	public void test3() {
		System.out.println();
		System.out.println("test 3 scheduling of job exactly between two other jobs (one time unit smaller than)");

		EquipletTest equiplet = new EquipletTest();
		// job in schedule
		equiplet.schedule(new Job(new Tick(0), new Tick(50)));
		equiplet.schedule(new Job(new Tick(100), new Tick(150)));

		// possible failure
		System.out.println("equiplet: " + equiplet);
		try {
			// possible failure
			equiplet.schedule(new Tick(51), new Tick(48));
			System.out.println("equiplet: " + equiplet);
		} catch (IllegalArgumentException e) {
			System.out.println("equiplet: " + equiplet);
			System.err.println("overlap in schedule incorrectly detected: " + e.getMessage());
			fail("overlap in schedule incorrectly detected");
		}
	}

	@Test
	public void test4() {
		System.out.println();
		System.out.println("test 4 scheduling of job adjacent at the begin");

		EquipletTest equiplet = new EquipletTest();
		// job in schedule
		equiplet.schedule(new Job(new Tick(0), new Tick(50)));
		equiplet.schedule(new Job(new Tick(100), new Tick(150)));

		try {
			// possible failure
			equiplet.schedule(new Tick(50), new Tick(49));
			System.out.println("equiplet: " + equiplet);
		} catch (IllegalArgumentException e) {
			System.out.println("equiplet: " + equiplet);
			System.err.println("overlap in schedule incorrectly detected: " + e.getMessage());
			fail("overlap in schedule incorrectly detected");
		}
	}

	@Test
	public void test5() {
		System.out.println();
		System.out.println("test 5 scheduling of job adjacent at the end");

		EquipletTest equiplet = new EquipletTest();
		// job in schedule
		equiplet.schedule(new Job(new Tick(0), new Tick(50)));
		equiplet.schedule(new Job(new Tick(100), new Tick(150)));

		try {
			// possible failure
			equiplet.schedule(new Tick(51), new Tick(49));
			System.out.println("equiplet: " + equiplet);
		} catch (IllegalArgumentException e) {
			System.out.println("equiplet: " + equiplet);
			System.err.println("overlap in schedule incorrectly detected: " + e.getMessage());
			fail("overlap in schedule incorrectly detected");
		}
	}

}
