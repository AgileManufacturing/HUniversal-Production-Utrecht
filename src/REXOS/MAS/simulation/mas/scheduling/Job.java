package simulation.mas.scheduling;

import java.util.Map;

public class Job implements Comparable<Job> {
	private double time;
	private double duration;
	private String productAgent;
	private String service;
	private Map<String, Object> criteria;

	public Job(double time, double duration, String productAgent,
			String service, Map<String, Object> criteria) {
		this.time = time;
		this.duration = duration;
		this.productAgent = productAgent;
		this.service = service;
		this.criteria = criteria;
	}

	@Override
	public int compareTo(Job job) {
		return time < job.time ? -1 : (time > job.time ? 1 : 0);
	}

	@Override
	public String toString() {
		return "Job [time=" + time + ", duration=" + duration + "]";
	}

	public double getTime() {
		return time;
	}

	public double getDuration() {
		return duration;
	}

	public String getProductAgent() {
		return productAgent;
	}
	
	public String getService(){
		return service;
	}
	
	public Map<String, Object> getCriteria(){
		return criteria;
	}
}
