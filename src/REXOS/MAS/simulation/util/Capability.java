package simulation.util;

import java.util.Map;

public class Capability {
	protected int id;
	public String service;
	private Map<String, Object> limitations;

	public Capability(int id, String service, Map<String, Object> limitations) {
		this.id = id;
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