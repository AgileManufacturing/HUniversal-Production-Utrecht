package HAL.steps;

import org.json.JSONObject;

public class ProductStep {
	protected String service;
	protected JSONObject criteria;
	
	public ProductStep(String service, JSONObject criteria) {
		this.service = service;
		this.criteria = criteria;
	}
	
	public String getService() {
		return service;
	}
	public JSONObject getCriteria() {
		return criteria;
	}
}
