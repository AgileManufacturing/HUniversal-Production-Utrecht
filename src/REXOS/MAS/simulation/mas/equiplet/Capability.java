package simulation.mas.equiplet;

import java.util.Map;

public class Capability {
	private String service;
	private Map<String, Object> limitations;

	public Capability(String service, Map<String, Object> limitations) {
		this.service = service;
		this.limitations = limitations;
	}

	public String getService() {
		return service;
	}

	public Map<String, Object> getLimitations() {
		return limitations;
	}

	@Override
	public String toString() {
		return "<" + service + "," + limitations + ">";
	}
}