package simulation.mas.equiplet;

import jade.core.AID;

import java.util.HashMap;
import java.util.Map;

import simulation.util.Tick;

public class Job implements Comparable<Job> {

	private AID product;
	private String productName;
	private String service;
	private Map<String, Object> criteria;
	private Tick start;
	private Tick due;
	private Tick deadline;
	private boolean ready;

	public Job(Tick start, Tick deadline) {
		this.product = null;
		this.productName = "";
		this.service = "";
		this.criteria = new HashMap<String, Object>();
		this.start = start;
		this.due = deadline;
		this.deadline = deadline;
		this.ready = false;
	}

	@Deprecated
	public Job(String service, String product, Map<String, Object> criteria, Tick start, Tick due, Tick deadline) {
		this.service = service;
		this.productName = product;
		this.criteria = criteria;
		this.start = start;
		this.due = due;
		this.deadline = deadline;
		this.ready = false;
	}

	public Job(AID product, String service, Map<String, Object> criteria, Tick start, Tick due, Tick deadline) {
		this.service = service;
		this.product = product;
		this.criteria = criteria;
		this.start = start;
		this.due = due;
		this.deadline = deadline;
		this.ready = false;
	}

	@Override
	public int compareTo(Job job) {
		return start.lessThan(job.start) ? -1 : (start.greaterThan(job.start) ? 1 : 0);
	}

	public String getService() {
		return service;
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

	public Map<String, Object> getCriteria() {
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
		return String.format("Job %s [product=%s, start=%s, due=%s, deadline=%s, ready=%s]", service, getProductAgentName(), start, due, deadline, ready);
	}
}
