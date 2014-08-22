package simulation.mas.equiplet;

import java.util.Map;

public class Capability {
	private String service;
	private Map<String, Object> limitations;
	private double duration;

	public Capability(String service, Map<String, Object> limitations, double duration) {
		this.service = service;
		this.limitations = limitations;
		this.duration = duration;
	}

	public String getService() {
		return service;
	}

	public Map<String, Object> getLimitations() {
		return limitations;
	}

	public double getDuration() {
		return duration;
	}
	
	@Override
	public String toString() {
		return "<" + service + "=" + duration + ", " + limitations + ">";
	}
}