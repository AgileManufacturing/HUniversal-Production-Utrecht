package simulation.config;

import java.util.HashMap;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;

/**
 * Data class for Product steps that are available to create products
 * 
 * name - the service name of the product step
 * probability - the probability that a product the product step needs for
 * his product
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(propOrder = { "name", "estimate" })
class ProductStepConfig {
	@XmlAttribute
	private int probability;

	@XmlElement(name = "name")
	private String name;

	@XmlElement(name = "estimate")
	private int estimate;

	public int getProbability() {
		return probability;
	}

	public String getService() {
		return name;
	}

	public int getEstimate() {
		return estimate;
	}

	public HashMap<String, Object> getCriteria() {
		return new HashMap<>();
	}

	public String toString() {
		return String.format("\n\t [name=%s, estimate=%d, criteria=%s)", name, estimate, getCriteria());
	}
}
