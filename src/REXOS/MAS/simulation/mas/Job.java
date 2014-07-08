package simulation.mas;

import java.util.HashMap;
import java.util.Map;

public class Job implements Comparable<Job> {

	private String service;
	private String product;
	private Map<String, Object> criteria;
	private double start;
	private double due;
	private double deadline;
	private boolean ready;

	public Job(double start, double deadline) {
		this.service = "";
		this.product = "";
		this.criteria = new HashMap<String, Object>();
		this.start = start;
		this.due = deadline;
		this.deadline = deadline;
		this.ready = false;
	}

	public Job(String service, String product, Map<String, Object> criteria, double start, double due, double deadline) {
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

	public String getProductAgent() {
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

	@Override
	public String toString() {
		return String.format("Job %s [product=%s, start=%.0f, due=%.0f, deadline=%.0f, ready=%s]", service, product, start, due, deadline, ready);
	}
}
