package simulation.mas.scheduling;

public class Node {

	private String equiplet;
	private double time;
	private double duration;

	public Node() {
		time = -1;
	}

	public Node(double time) {
		this.equiplet = null;
		this.time = time;
		this.duration = 0;
	}

	public Node(String equiplet, double time, double duration) {
		this.equiplet = equiplet;
		this.time = time;
		this.duration = duration;
	}

	public String getEquiplet() {
		return equiplet;
	}

	public double getTime() {
		return time;
	}

	public double getDuration() {
		return duration;
	}

	@Override
	public String toString() {
		if (equiplet != null) {
			return String.format("N(%s), %.2f %.2f", equiplet, time, duration);
		} else if (time > -1) {
			return String.format("N(source), %.2f", time);
		} else {
			return "N(sink)";
		}
	}
}
