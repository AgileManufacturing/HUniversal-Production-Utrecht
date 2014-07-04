package simulation.config;

import java.util.HashMap;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;

/**
 * Data class for Capabilty configurations
 * 
 * name - capability name
 * duration - average duration of executing the capability
 * limitations - list of limitations, key and value, of the equiplet capable
 * to perform the capability
 */
@XmlRootElement
@XmlType(propOrder = { "name", "duration", "limitations" })
class CapabilityConfig {
	@XmlElement(name = "name")
	private String name;

	@XmlElement(name = "duration")
	private DurationConfig duration;

	@XmlElement(name = "limitation")
	private HashMap<String, Object> limitations = null;

	public String getName() {
		return name;
	}

	public DurationType getDurationType() {
		return duration.type();
	}

	public double getDuration() {
		return duration.mean();
	}

	public String toString() {
		return String.format("%s [duration=%s (%s), limitations=%s]", name, duration.mean(), duration.type(), limitations);
	}
}