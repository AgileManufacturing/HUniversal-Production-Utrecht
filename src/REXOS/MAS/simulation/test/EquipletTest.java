package MAS.simulation.test;

import jade.core.AID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import org.json.JSONObject;

import MAS.equiplet.Capability;
import MAS.equiplet.EquipletAgent;
import MAS.equiplet.Job;
import MAS.util.Pair;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Tuple;

class EquipletTest extends EquipletAgent {
	private static final long serialVersionUID = 1L;
	private Map<String, Tick> pTimes = new HashMap<String, Tick>();

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

	protected boolean executed(Job job) {
		return history.add(job);
	}

	@Override
	public boolean isCapable(String service, JSONObject criteria) {
		return pTimes.containsKey(service);
	}

	@Override
	public Tick estimateService(String service) {
		return pTimes.get(service);
	}

	protected boolean schedule(Tick start, Tick duration) {
		String uuid = UUID.randomUUID().toString();
		pTimes.put(uuid, duration);

		List<Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>> requests = new ArrayList<>();
		requests.add(new Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>(0, new Pair<Tick, Tick>(start, start.add(duration)), uuid, new JSONObject()));

		return super.schedule(new AID("Test Equiplet", AID.ISGUID), requests);
	}

	@Override
	public double load(Tick time, Tick window) {
		return super.load(time, window);
	}

	@Override
	public double loadHistory(Tick time, Tick window) {
		return super.loadHistory(time, window);
	}

	public void history(Job job) {
		history.add(job);
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
