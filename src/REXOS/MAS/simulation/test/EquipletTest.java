package simulation.test;

import java.util.ArrayList;
import java.util.List;

import simulation.mas.equiplet.Capability;
import simulation.mas.equiplet.EquipletAgent;
import simulation.mas.equiplet.Job;
import simulation.util.Pair;
import simulation.util.Position;
import simulation.util.Tick;

class EquipletTest extends EquipletAgent {
	private static final long serialVersionUID = 1L;

	public EquipletTest() {
		init(new Position(-1, -1), new ArrayList<Capability>());
	}

	@Override
	protected void execute(Job job) {
		executing = job;
	}

	protected boolean schedule(Job job) {
		return schedule.add(job);
	}

	@Override
	public double load(Tick time, Tick window) {
		return super.load(time, window);
	}

	@Override
	public List<Pair<Tick, Tick>> available(Tick time, Tick duration, Tick deadline) {
		return super.available(time, duration, deadline);
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
}
