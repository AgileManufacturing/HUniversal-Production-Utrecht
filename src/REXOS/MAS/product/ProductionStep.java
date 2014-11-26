package MAS.product;

import jade.core.AID;

import java.util.Map;

import MAS.util.Position;
import MAS.util.Tick;

public class ProductionStep {

	private ProductStep productStep;
	private AID equiplet;
	private String equipletName;
	private Position position;
	private Tick time;
	private Tick duration;

	/**
	 * 
	 * @param productStep
	 *            the product step
	 * @param equilet
	 *            the executing equiplet of the production step
	 * @param position
	 *            the position of the equiplet
	 * @param time
	 *            the start time of the product step
	 * @param duration
	 *            the estimate duration of the product step
	 */
	public ProductionStep(ProductStep productStep, AID equilet, Position position, Tick time, Tick duration) {
		this.productStep = productStep;
		this.equiplet = equilet;
		this.position = position;
		this.time = time;
		this.duration = duration;
	}

	@Deprecated
	public ProductionStep(ProductStep productStep, String equipletName, Position position, Tick time, Tick duration) {
		this.productStep = productStep;
		this.equipletName = equipletName;
		this.time = time;
		this.duration = duration;
	}

	/**
	 * @return the product step
	 */
	public ProductStep getProductStep() {
		return productStep;
	}

	/**
	 * @return the address of the equiplet
	 */
	public AID getEquiplet() {
		return equiplet;
	}

	/**
	 * @return the name of the equiplet
	 */
	public String getEquipletName() {
		if (equiplet != null) {
			return equiplet.getLocalName();
		} else {
			return equipletName;
		}
	}

	/**
	 * @return the index of the product step
	 */
	public int getIndex() {
		return productStep.getIndex();
	}

	/**
	 * {@link ProductionStep#position}
	 * 
	 * @return the position of the equiplet
	 */
	public Position getPosition() {
		return position;
	}

	/**
	 * {@link ProductionStep#time}
	 * 
	 * @return the scheduled start time of the product step
	 */
	public Tick getStart() {
		return time;
	}

	/**
	 * {@link ProductionStep#time}
	 * 
	 * @param time
	 *            the new updated time of the product step
	 */
	public void updateStart(Tick time) {
		this.time = time;
	}

	/**
	 * {@link ProductionStep#duration}
	 * 
	 * @return the estimate duration of the product step
	 */
	public Tick getDuration() {
		return duration;
	}

	/**
	 * @return the service of the product step
	 */
	public String getService() {
		return productStep.getService();
	}

	/**
	 * @return the criteria of the product step
	 */
	public Map<String, Object> getCriteria() {
		return productStep.getCriteria();
	}

	@Override
	public String toString() {
		return "Production Step(" + productStep.getService() + "," + getEquipletName() + ", " + getIndex() + ", from=" + time + ", until=" + time.add(duration) + ")";
	}
}
