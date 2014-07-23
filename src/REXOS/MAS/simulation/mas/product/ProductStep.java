package simulation.mas.product;

import java.util.Map;

public class ProductStep {

	private int index;
	private String service;
	private Map<String, Object> criteria;

	public ProductStep(int index, String service, Map<String, Object> criteria) {
		this.index = index;
		this.service = service;
		this.criteria = criteria;
	}

	public int getIndex() {
		return index;
	}

	public String getService() {
		return service;
	}

	public Map<String, Object> getCriteria() {
		return criteria;
	}

	@Override
	public String toString() {
		return String.format("(%d=%s,%s)", index, service, criteria);
	}
}