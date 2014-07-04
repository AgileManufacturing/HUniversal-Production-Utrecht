package simulation.mas.scheduling;

public class Job implements Comparable<Job> {
	private double time;
	private double duration;
	private String productAgent;

	public Job(double time, double duration) {
		this.time = time;
		this.duration = duration;
	}

	public Job(double time, double duration, String productAgent) {
		this.time = time;
		this.duration = duration;
		this.productAgent = productAgent;
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
}
