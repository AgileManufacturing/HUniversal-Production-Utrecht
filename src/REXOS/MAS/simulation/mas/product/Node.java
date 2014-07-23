package simulation.mas.product;

import jade.core.AID;

public class Node {

	private AID equipletAID;
	private String equiplet;
	private double time;
	private double duration;

	public Node() {
		this.equipletAID = null;
		this.time = -1;
		this.duration = 0;
	}

	public Node(double time) {
		this.equipletAID = null;
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
	public boolean equals(Object obj) {
		if (!(obj instanceof Node)) {
			return false;
		}
		if (obj == this) {
			return true;
		}
		Node node = (Node) obj;
		if (equipletAID == null || node.getEquipletAID() == null) {
			return false;
		}
		
		return equipletAID.equals(node.getEquipletAID()) && time == node.getTime() && duration == node.getDuration();
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
