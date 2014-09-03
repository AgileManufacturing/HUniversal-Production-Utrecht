package simulation.config;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlElementWrapper;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;

/**
 * Data class for Equiplet configurations
 * 
 * name - Equiplet name
 * position - Equiplet position is the grid
 * capabilities - list of the equiplets capabilities
 */
@XmlRootElement
@XmlType(propOrder = { "name", "position", "breakdown", "repaired", "capabilities" })
class EquipletConfig {
	@XmlElement(name = "name")
	private String name;

	@XmlElement(name = "position")
	private PositionConfig position;

	@XmlElement(name = "breakdown")
	private DurationConfig breakdown;

	@XmlElement(name = "repaired")
	private DurationConfig repaired;

	// @XmlElement(name = "capabilities")
	private List<CapabilityConfig> capabilities;

	public EquipletConfig() {

	}

	public EquipletConfig(String name, PositionConfig position, DurationConfig breakdown, DurationConfig repaired, List<CapabilityConfig> capabilities) {
		this.name = name;
		this.position = position;
		this.breakdown = breakdown;
		this.repaired = repaired;
		this.capabilities = capabilities;
	}

	public String getName() {
		return name;
	}

	public PositionConfig getPosition() {
		return position;
	}

	public DurationConfig getBreakdown() {
		return breakdown;
	}

	public DurationConfig getRepaired() {
		return repaired;
	}

	@XmlElementWrapper(name = "capabilities")
	@XmlElement(name = "capability", type = CapabilityConfig.class)
	public void setCapabilities(List<CapabilityConfig> capabilities) {
		this.capabilities = capabilities;
	}

	public List<CapabilityConfig> getCapabilities() {
		return capabilities;
	}

	public String toString() {
		return String.format("\n\t%s [pos=%s, capabilities=%s]", name, position, capabilities);
	}
}
