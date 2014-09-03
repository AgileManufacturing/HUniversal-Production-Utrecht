package simulation.config;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlValue;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlRootElement
class DurationConfig {
	@XmlAttribute
	private String type;

	@XmlAttribute
	private int location;

	@XmlAttribute
	private int scale;

	@XmlAttribute
	private int shape;

	@XmlValue
	private double time;

	public DurationConfig() {

	}

	public DurationConfig(String type, double time) {
		this.type = type;
		this.time = time;
	}

	public double mean() {
		return time;
	}

	public double location() {
		return location;
	}

	public double scale() {
		return scale;
	}

	public double shape() {
		return shape;
	}

	public DurationType type() {
		if (type.equalsIgnoreCase("exp")) {
			return DurationType.EXP;
		} else if (type.equalsIgnoreCase("normal")) {
			return DurationType.NORMAL;
		} else if (type.equalsIgnoreCase("weibull")) {
			return DurationType.WEIBULL;
		} else {
			return DurationType.DETERMINISTIC;
		}
	}
}