package MAS.equiplet;

import java.util.Map;

import org.apache.commons.lang.builder.HashCodeBuilder;

import MAS.util.Tick;

public class Capability {
	public String name;
	private String service;
	private Map<String, Object> limitations;
	private Tick duration;
	
	public Capability(String service, Map<String, Object> limitations, Tick duration) {
		this.name = service;
		this.service = service;
		this.limitations = limitations;
		this.duration = duration;
	}

	public Capability(String capability, String service, Map<String, Object> limitations, Tick duration) {
		this.name = capability;
		this.service = service;
		this.limitations = limitations;
		this.duration = duration;
	}

	public String getService() {
		return service;
	}
	
	public String getName() {
		return name;
	}

	public Map<String, Object> getLimitations() {
		return limitations;
	}

	public Tick getDuration() {
		return duration;
	}

	@Override
	public int hashCode() {
		return new HashCodeBuilder(41, 59).append(service).append(limitations).toHashCode();
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
		Capability capability = (Capability) obj;
		return service.equals(capability.getService()) && limitations.equals(capability.getLimitations());
	}

	@Override
	public String toString() {
		return "<" + service + "=" + duration + ", " + limitations + ">";
	}
}