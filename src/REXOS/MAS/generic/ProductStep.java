package generic;

import HAL.Capability;

import com.google.gson.JsonObject;

/**
 * A ProductStep is a step that is has not been translated and is supplied by the {@link ProductAgent}. 
 * It only describes a desired mutation (using criteria) for the product and does not concern itself with the realization of this mutation.
 * ProductSteps are supplied by the {@link ProductAgent} and interpeted by {@link Capability}.
 * @author Tommas Bakker
 *
 */
public class ProductStep {
	public static final String TARGET = "target";
	public static final String SUBJECTS = "subjects";

	public static final String CRITERIA = "criteria";
	public static final String SERVICE = "service";
	public static final String ID = "id";
	
	
	private Service service;
	private JsonObject criteria;
	private String id;
	
	public ProductStep(JsonObject json){
		if (json.has(ID)){
			this.id = json.get(ID).getAsString();
		}
		if (json.has(SERVICE)){
			this.service = new Service(json.get(SERVICE).getAsString());
		}
		if (json.has("CRITERIA")){
			this.criteria = json.get("CRITERIA").getAsJsonObject();
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
		    " " + ID + ":" + id + ",\n" +
		    " " + SERVICE + ":" + service.toJSON() + ",\n" +
		    " " + CRITERIA + ": {" + criteria + "}\n" +
		    "}";
	}
}
