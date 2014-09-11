package MAS.simulation.config;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;

/**
 * Data class for Position of the equiplet
 * 
 * x - x coordinate
 * y - y coordinate
 */
@XmlRootElement
@XmlType(propOrder = { "x", "y" })
class PositionConfig {
	@XmlElement(name = "x")
	private int x;

	@XmlElement(name = "y")
	private int y;

	public PositionConfig() {

	}

	public PositionConfig(int x, int y) {
		this.x = x;
		this.y = y;
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public String toString() {
		return String.format("(%d,%d)", x, y);
	}
}
