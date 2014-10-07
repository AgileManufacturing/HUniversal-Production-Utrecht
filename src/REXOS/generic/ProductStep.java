package generic;

import org.json.JSONObject;

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
	private JSONObject criteria;
	private int id;
	
	public ProductStep(JSONObject json){
		if (json.has(ID)){
			this.id = json.optInt(ID);
		}
		if (json.has(SERVICE)){
			this.service = new Service(json.optString(SERVICE));
		}
		if (json.has("CRITERIA")){
			this.criteria = json.optJSONObject("CRITERIA");
		}
	}
	
	public ProductStep(int productStepId, JSONObject criteria, Service service){
		this.id = productStepId;
		this.criteria = criteria;
		this.service = service;
	}
	
	public Service getService(){
		return this.service;
	}
	public JSONObject getCriteria(){
		return this.criteria;
	}
	public int getId(){
		return this.id;
	}
	
	public String toJSON(){
		  return  "{" +
		    " \"" + ID + "\":" + id + ",\n" +
		    " \"" + SERVICE + "\":" + service.toJSON() + ",\n" +
		    " \"" + CRITERIA + "\": {" + criteria + "}\n" +
		    "}";
	}
	
	public String toString() {
		return toJSON();
	}
}
