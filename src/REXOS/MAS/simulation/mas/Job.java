package simulation.mas;

import jade.core.AID;

import java.util.HashMap;
import java.util.Map;

public class Job implements Comparable<Job> {

	private AID product;
	private String productName;
	private String service;
	private Map<String, Object> criteria;
	private double start;
	private double due;
	private double deadline;
	private boolean ready;

	public Job(double start, double deadline) {
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
	public Job(String service, String product, Map<String, Object> criteria, double start, double due, double deadline) {
		this.service = service;
		this.productName = product;
		this.criteria = criteria;
		this.start = start;
		this.due = due;
		this.deadline = deadline;
		this.ready = false;
	}

	public Job(AID product, String service, Map<String, Object> criteria, double start, double due, double deadline) {
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
		return start < job.start ? -1 : (start > job.start ? 1 : 0);
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

	public double getStartTime() {
		return start;
	}

	public double getDueTime() {
		return due;
	}

	public double getDeadline() {
		return deadline;
	}

	public double getDuration() {
		return due - start;
	}

	public boolean isReady() {
		return ready;
	}

	public void setReady() {
		this.ready = true;
	}

	public void updateStartTime(double time) {
		this.start = time;
	}

	public void updateDueTime(double time) {
		this.due = time;
	}

	@Override
	public String toString() {
		return String.format("Job %s [product=%s, start=%.0f, due=%.0f, deadline=%.0f, ready=%s]", service, product, start, due, deadline, ready);
	}
}
