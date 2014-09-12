package simulation.config;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlElementWrapper;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;

/**
 * Data class to store products configuration
 * 
 * arrival - average arrival time of products
 * steps - average product steps a product has when created
 * 
 */
@XmlRootElement
@XmlType(propOrder = { "arrival", "deadline", "path", "steps" })
class ProductsConfig {
	@XmlElement(name = "arrival")
	private DurationConfig arrival;

	@XmlElement(name = "deadline")
	private DurationConfig deadline;

	@XmlElement(name = "path")
	private int path;

	private List<ProductStepConfig> steps;

	public double getArrival() {
		return arrival.mean();
	}

	public DurationType getArrivalType() {
		return arrival.type();
	}

	public double getDeadline() {
		return deadline.mean();
	}

	public DurationType getDeadlineType() {
		return deadline.type();
	}

	public int getPathLength() {
		return path;
	}

	@XmlElementWrapper(name = "steps")
	@XmlElement(name = "step", type = ProductStepConfig.class)
	public void setSteps(List<ProductStepConfig> data) {
		this.steps = data;
	}

	public List<ProductStepConfig> getSteps() {
		return steps;
	}

	public String toString() {
		return String.format("[arrival=%.0f (%s), deadline=%.0f (%s), path length=steps=%d, product steps=%s]", arrival.mean(), arrival.type(), deadline.mean(), deadline.type(), path, steps);
	}
}
