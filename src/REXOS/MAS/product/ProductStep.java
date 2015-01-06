package MAS.product;

import org.json.JSONObject;

public class ProductStep {

	private int index;
	private String service;
	private JSONObject criteria;

	public ProductStep(int index, String service, JSONObject criteria) {
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

	public JSONObject getCriteria() {
		return criteria;
	}

	@Override
	public String toString() {
		return String.format("(%d=%s,%s)", index, service, criteria);
	}

	public void updateCriteria(JSONObject criteria) {
		this.criteria = criteria;
	}
}