package simulation.util;

import jade.core.AID;

import java.util.Map;

public class ProductionStep {

	private ProductStep productStep;
	private AID equiplet;
	private String equipletName;
	private double time;
	private double duration;

	public ProductionStep(ProductStep productStep, AID equilet, double time, double duration) {
		this.productStep = productStep;
		this.equiplet = equilet;
		this.time = time;
		this.duration = duration;
	}

	@Deprecated
	public ProductionStep(ProductStep productStep, String equipletName, double time, double duration) {
		this.productStep = productStep;
		this.equipletName = equipletName;
		this.time = time;
		this.duration = duration;
	}

	public ProductStep getProductStep() {
		return productStep;
	}

	public AID getEquiplet() {
		return equiplet;
	}

	public String getEquipletName() {
		if (equiplet != null) {
			return equiplet.getLocalName();
		} else {
			return equipletName;
		}

	}

	public double getTime() {
		return time;
	}

	public double getDuration() {
		return duration;
	}

	public String getService() {
		return productStep.getService();
	}

	public Map<String, Object> getCriteria() {
		return productStep.getCriteria();
	}

	@Override
	public String toString() {
		return "Production Step(" + productStep.getService() + "," + equiplet + "," + time + "," + duration + ")";
	}
}
