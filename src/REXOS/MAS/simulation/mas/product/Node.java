package MAS.simulation.mas.product;

import org.apache.commons.lang.builder.HashCodeBuilder;

import MAS.simulation.util.Tick;
import jade.core.AID;

public class Node {

	private AID equipletAID;
	private String equiplet;
	private int index;
	private Tick time;
	private Tick duration;

	public Node() {
		this.equipletAID = null;
		this.index = -1;
		this.time = new Tick(-1);
		this.duration = new Tick(0);
	}

	public Node(Tick time) {
		this.equipletAID = null;
		this.index = -1;
		this.time = time;
		this.duration = new Tick(0);
	}

	@Deprecated
	public Node(String equiplet, Tick time, Tick duration, int index) {
		this.equiplet = equiplet;
		this.index = index;
		this.time = time;
		this.duration = duration;
	}

	public Node(AID equiplet, Tick time, Tick duration, int index) {
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

	public Tick getTime() {
		return time;
	}

	public Tick getDuration() {
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
			return String.format("N(%s), %d, %s, %s", equipletAID.getLocalName(), index, time, duration);
		} else if (equiplet != null) {
			return String.format("N(%s), %d, %s, %s", equiplet, index, time, duration);
		} else if (time.greaterThan(-1)) {
			return String.format("N(source), %s", time);
		} else {
			return "N(sink)";
		}
	}
}
