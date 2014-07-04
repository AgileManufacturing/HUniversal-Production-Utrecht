package simulation.util;

import java.util.HashMap;


public class ProductionStep {

	private ProductStep productStep;
	private String equiplet;
	private double time;
	private double duration;

	public ProductionStep(ProductStep productStep, String equilet, double time, double duration) {
		this.productStep = productStep;
		this.equiplet = equilet;
		this.time = time;
		this.duration = duration;
	}

	public ProductStep getProductStep() {
		return productStep;
	}

	public String getEquiplet() {
		return equiplet;
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
	
	public HashMap<String, Object> getCriteria() {
		return productStep.getCriteria();
	}

	@Override
	public String toString() {
		return "Production Step(" + productStep.getService() + "," + equiplet + "," + time + "," + duration + ")";
	}
}
