package MAS.equiplet;

import jade.core.AID;

import org.apache.commons.lang.builder.HashCodeBuilder;
import org.json.JSONObject;

import MAS.util.Tick;

public class Job implements Comparable<Job> {

	private boolean dummy;
	private AID product;
	private String productName;
	private int index;
	private String service;
	private JSONObject criteria;
	private Tick start;
	private Tick due;
	private Tick deadline;
	private boolean ready;

	public Job(Tick start, Tick deadline) {
		this.dummy = true;
		this.product = null;
		this.productName = "";
		this.service = "";
		this.index = -1; 
		this.criteria = new JSONObject();
		this.start = start;
		this.due = deadline;
		this.deadline = deadline;
		this.ready = false;
	}

	@Deprecated
	public Job(int index, String service, String product, JSONObject criteria, Tick start, Tick due, Tick deadline) {
		this.dummy = false;
		this.index = index;
		this.service = service;
		this.productName = product;
		this.criteria = criteria;
		this.start = start;
		this.due = due;
		this.deadline = deadline;
		this.ready = false;
	}

	public Job(int index, AID product, String service, JSONObject criteria, Tick start, Tick due, Tick deadline) {
		this.dummy = false;
		this.index = index;
		this.service = service;
		this.product = product;
		this.productName = product.getLocalName();
		this.criteria = criteria;
		this.start = start;
		this.due = due;
		this.deadline = deadline;
		this.ready = false;
	}

	@Override
	public int hashCode() {
		// two randomly chosen prime numbers
		return new HashCodeBuilder(41, 67).append(productName).append(index).toHashCode();
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null) {
			return false;
		}
		if (obj == this) {
			return true;
		}
		if (obj.getClass() != getClass()) {
			return false;
		}
		Job job = (Job) obj;
		return job.getProductAgent().equals(getProductAgent()) && job.getIndex() == index;
	}

	@Override
	public int compareTo(Job job) {
		// System.out.println(this + " COMPARE TO " + job + "==");
		if (!dummy && !job.dummy && job.equals(this)) {
			return 0;
		}

		// other ends before or when this starts
		if (job.getDue() != null && (job.getDue().equals(start) || job.getDue().lessThan(start))) {
			return 1;
		}
		// other starts after or when this ends
		if (due != null && (job.getStartTime().equals(due) || job.getStartTime().greaterThan(due))) {
			return -1;
		}
		return 0;
	}

	public String getProductAgentName() {
		if (product != null) {
			return product.getLocalName();
		} else {
			return productName;
		}
	}

	public AID getProductAgent() {
		return product;
	}

	public int getIndex() {
		return index;
	}

	public String getService() {
		return service;
	}

	public JSONObject getCriteria() {
		return criteria;
	}

	public Tick getStartTime() {
		return start;
	}

	public Tick getDue() {
		return due;
	}

	public Tick getDeadline() {
		return deadline;
	}

	public Tick getDuration() {
		return due.minus(start);
	}

	public boolean isReady() {
		return ready;
	}

	public void setReady() {
		this.ready = true;
	}

	public void updateStartTime(Tick time) {
		this.due = time.add(due.minus(start));
		this.start = time;
	}

	public void updateDueTime(Tick time) {
		this.due = time;
	}

	@Override
	public String toString() {
		return String.format("Job %s [product=%s, index=%d, start=%s, due=%s, deadline=%s, ready=%s]", service, getProductAgentName(), index, start, due, deadline, ready);
	}
}
