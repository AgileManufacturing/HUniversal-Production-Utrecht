package HAL.steps;

import java.io.Serializable;

import HAL.Module;
import HAL.Service;
import HAL.capabilities.Capability;

import com.google.gson.JsonObject;

/**
 * A ProductStep is a step that is has not been translated and is supplied by the {@link ProductAgent}. 
 * It only describes a desired mutation (using criteria) for the product and does not concern itself with the realization of this mutation.
 * ProductSteps are supplied by the {@link ProductAgent} and interpeted by {@link Capability}.
 * @author Tommas Bakker
 *
 */
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
