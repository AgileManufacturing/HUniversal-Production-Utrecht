package simulation.util;

import java.util.HashMap;

public class ProductStep {

	private String service;
	private HashMap<String, Object> criteria;

	public ProductStep(String service, HashMap<String, Object> criteria, double duration) {
		this.service = service;
		this.criteria = criteria;
	}

	public String getService() {
		return service;
	}

	public HashMap<String, Object> getCriteria() {
		return criteria;
	}

	@Override
	public String toString() {
		return "(" + service + "," + criteria + ")";
	}
}