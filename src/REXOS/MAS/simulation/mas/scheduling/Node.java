package simulation.mas.scheduling;

import jade.core.AID;

public class Node {

	private AID equipletAID;
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

	@Deprecated
	public Node(String equiplet, double time, double duration) {
		this.equiplet = equiplet;
		this.time = time;
		this.duration = duration;
	}
	
	public Node(AID equiplet, double time, double duration) {
		this.equipletAID = equiplet;
		this.time = time;
		this.duration = duration;
	}

	public String getEquiplet() {
		return equiplet;
	}

	public AID getEquipletAID() {
		return equipletAID;
	}

	public double getTime() {
		return time;
	}

	public double getDuration() {
		return duration;
	}

	@Override
	public String toString() {
		if (equipletAID != null) {
			return String.format("N(%s), %.2f, %.2f", equipletAID.getLocalName(), time, duration);
		} else if (equiplet != null) {
			return String.format("N(%s), %.2f, %.2f", equiplet, time, duration);
		} else if (time > -1) {
			return String.format("N(source), %.2f", time);
		} else {
			return "N(sink)";
		}
	}
}
