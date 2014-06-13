package generic;

import HAL.capabilities.Capability;

import com.google.gson.JsonObject;

/**
 * A ProductStep is a step that is has not been translated and is supplied by the {@link ProductAgent}. 
 * It only describes a desired mutation (using criteria) for the product and does not concern itself with the realization of this mutation.
 * ProductSteps are supplied by the {@link ProductAgent} and interpeted by {@link Capability}.
 * @author Tommas Bakker
 *
 */
public class ProductStep {
	private Service service;
	private JsonObject criteria;
	private String id;
	
	public ProductStep(JsonObject json){
		if (json.has("id")){
			this.id = json.get("id").getAsString();
		}
		if (json.has("service")){
			this.service = new Service(json.get("service").getAsString());
		}
		if (json.has("criteria")){
			this.criteria = json.get("criteria").getAsJsonObject();
		}
	}
	
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
	
	public String toJSON(){
		  return  "{" +
		    " id:" + id + ",\n" +
		    " service:" + service.toJSON() + ",\n" +
		    " criteria: {" + criteria + "}\n" +
		    "}";
	}
}
