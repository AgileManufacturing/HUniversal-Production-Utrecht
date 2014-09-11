package simulation.mas.equiplet;

import java.util.Map;

import simulation.util.Tick;

public class Capability {
	private String service;
	private Map<String, Object> limitations;
	private Tick duration;

	public Capability(String service, Map<String, Object> limitations, Tick duration) {
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

	public Tick getDuration() {
		return duration;
	}
	
	@Override
	public String toString() {
		return "<" + service + "=" + duration + ", " + limitations + ">";
	}
}