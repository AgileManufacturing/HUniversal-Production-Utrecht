package simulation.mas.product;

import org.apache.commons.lang.builder.HashCodeBuilder;

import jade.core.AID;

public class Node {

	private AID equipletAID;
	private String equiplet;
	private int index;
	private double time;
	private double duration;

	public Node() {
		this.equipletAID = null;
		this.index = -1;
		this.time = -1;
		this.duration = 0;
	}

	public Node(double time) {
		this.equipletAID = null;
		this.index = -1;
		this.time = time;
		this.duration = 0;
	}

	@Deprecated
	public Node(String equiplet, double time, double duration, int index) {
		this.equiplet = equiplet;
		this.index = index;
		this.time = time;
		this.duration = duration;
	}

	public Node(AID equiplet, double time, double duration, int index) {
		this.equipletAID = equiplet;
		this.index = index;
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
	public int hashCode() {
		return new HashCodeBuilder(43, 67).append(equipletAID).append(time).append(duration).append(index).toHashCode();
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
		
		return equipletAID.equals(node.getEquipletAID()) && time == node.getTime() && duration == node.getDuration() && index == node.index;
	}

	@Override
	public String toString() {
		if (equipletAID != null) {
			return String.format("N(%s), %d, %.2f, %.2f", equipletAID.getLocalName(), index, time, duration);
		} else if (equiplet != null) {
			return String.format("N(%s), %d, %.2f, %.2f", equiplet, index, time, duration);
		} else if (time > -1) {
			return String.format("N(source), %.2f", time);
		} else {
			return "N(sink)";
		}
	}
}
