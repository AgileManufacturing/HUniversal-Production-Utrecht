package HAL.steps;

import java.io.Serializable;

import HAL.Service;

import com.google.gson.JsonObject;

public class ProductStep implements Serializable {
	private static final long serialVersionUID = 6788269249283740246L;
	private Service service;
	private JsonObject criteria;
	private String id;
	
	public ProductStep(String productStepId, JsonObject criteria, Service service){
		this.id = productStepId;
		this.criteria = criteria;
		this.service = service;
	}
	
	public Service getService(){
		return this.service;
	}
	public JsonObject getCriteria(){
		return this.criteria;
	}
	public String getId(){
		return this.id;
	}
}
