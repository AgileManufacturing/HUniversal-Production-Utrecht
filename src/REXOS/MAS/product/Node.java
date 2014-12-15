package MAS.product;

import jade.core.AID;

import org.apache.commons.lang.builder.HashCodeBuilder;

import MAS.util.Tick;

public class Node implements Comparable<Node> {

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
		return equipletAID != null ? equipletAID.getLocalName() : equiplet;
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

	public int getIndex() {
		return index;
	}

	@Override
	public int hashCode() {
		return new HashCodeBuilder(47, 67).append(equipletAID).append(duration).append(index).toHashCode();
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

		return (equipletAID != null && equipletAID.equals(node.getEquipletAID()) || equiplet != null && equiplet.equals(node.getEquiplet())) && duration.equals(node.getDuration())
				&& index == node.index;
	}

	@Override
	public String toString() {
		if (equipletAID != null) {
			return String.format("N(%s), %d, %s, %s", equipletAID.getLocalName(), index, time, duration);
			//return String.format("N(%s)", equipletAID.getLocalName());
		} else if (equiplet != null) {
			return String.format("N(%s), %d, %s, %s", equiplet, index, time, duration);
		} else if (time.greaterThan(-1)) {
			return String.format("N(source), %s", time);
		} else {
			return "N(sink)";
		}
	}

	@Override
	public int compareTo(Node o) {
		return index - o.index;
	}
}
