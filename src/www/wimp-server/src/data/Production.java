package data;

import java.util.ArrayList;

public class Production {

	private Long id;
	private ArrayList<ProductionStep> productionSteps;

	public Long getId() {
		return id;
	}

	public void setId(Long id) {
		this.id = id;
	}

	public ArrayList<ProductionStep> getProductionStep() {
		return productionSteps;
	}

	public void setProductionStep(ArrayList<ProductionStep> productionStep) {
		this.productionSteps = productionStep;
	}

	public String toString() {
		return "";
	}
}
