package MAS.simulation.offline;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import MAS.simulation.util.Pair;
import MAS.simulation.util.Triple;

class Equiplet {
	private String name;
	private List<String> services;
	private Position position;
	private EquipletState state;
	private Queue<Pair<Integer, ProductStep>> queue;
	private Pair<Integer, ProductStep> executingStep;
	private int executed;

	private Triple<Double, Double, Double> history;
	private double lastHistoryUpdate;

	// simulation depended variables
	private double shouldFinish;
	private double timeBroken;

	public Equiplet(String name, List<String> services, Position position) {
		this.name = name;
		this.services = services;
		this.position = position;
		this.state = EquipletState.IDLE;
		this.queue = new LinkedList<>();
		this.executingStep = null;
		this.executed = 0;
		this.lastHistoryUpdate = 0;
		this.history = new Triple<Double, Double, Double>(0d, 0d, 0d);

		this.shouldFinish = -1;
		this.timeBroken = -1;
	}

	public String getName() {
		return name;
	}

	public EquipletState getState() {
		return state;
	}

	public List<String> getServices() {
		return services;
	}

	public Position getPosition() {
		return position;
	}

	public boolean isCapable(String service) {
		return services.contains(service);
	}

	public int getWaiting() {
		return queue.size();
	}

	public boolean addJob(double time, int product, ProductStep step) {
		historyUpdate(time);
		if (state == EquipletState.IDLE) {
			executingStep = new Pair<Integer, ProductStep>(product, step);
			state = EquipletState.BUSY;
			return true;
		} else {
			queue.add(new Pair<Integer, ProductStep>(product, step));
			return false;
		}
	}

	public Pair<Integer, Boolean> currentJobFinished(double time) {
		historyUpdate(time);
		Pair<Integer, ProductStep> step = executingStep;
		boolean empty = queue.isEmpty();
		executingStep = empty ? null : queue.poll();
		state = empty ? EquipletState.IDLE : state;
		executed++;

		return new Pair<Integer, Boolean>(step.first, !empty);
	}

	public ProductStep getCurrentProductStep() {
		return executingStep.second;
	}

	public void setShouldFinish(double time) {
		this.shouldFinish = time;
		this.state = EquipletState.WAS_BROKEN;
	}

	public boolean hasFinished() {
		return shouldFinish > 0;
	}

	public double getShouldFinish() {
		return shouldFinish;
	}

	public void setTimeBroken(double time) {
		historyUpdate(time);
		this.state = EquipletState.BROKEN;
		this.timeBroken = time;
	}

	public double continueAfterBreakdown(double time) {
		historyUpdate(time);
		state = EquipletState.BUSY;
		shouldFinish = -1;
		return timeBroken;
	}

	public void setRepaired(double time) {
		historyUpdate(time);
		state = executingStep == null ? EquipletState.IDLE : EquipletState.WAS_BROKEN;
	}

	public int executedJobs() {
		return executed;
	}

	private void historyUpdate(double time) {
		double elapsed = time - lastHistoryUpdate;
		lastHistoryUpdate = time;
		if (state == EquipletState.IDLE) {
			history.first += elapsed;
		} else if (state == EquipletState.BUSY || state == EquipletState.WAS_BROKEN) {
			history.second += elapsed;
		} else if (state == EquipletState.BROKEN) {
			history.third += elapsed;
		}
	}

	public Triple<Double, Double, Double> getHistory(double time) {
		historyUpdate(time);
		return history;
	}

	@Override
	public String toString() {
		if (state == EquipletState.BROKEN) {
			return String.format("%s:[state=%s, services=%s, queue=%s, time broken=%s]", name, state, Arrays.toString(services.toArray()), Arrays.toString(queue.toArray()), timeBroken);
		} else if (state == EquipletState.WAS_BROKEN) {
			return String.format("%s:[state=%s, services=%s, queue=%s, time broken=%s, should finished=%s]", name, state, Arrays.toString(services.toArray()), Arrays.toString(queue.toArray()), timeBroken, shouldFinish);
		} else {
			return String.format("%s:[state=%s, services=%s, executing=%s, queue=%s]", name, state, Arrays.toString(services.toArray()), (state == EquipletState.IDLE ? "null" : executingStep), Arrays.toString(queue.toArray()));
		}
	}
}
