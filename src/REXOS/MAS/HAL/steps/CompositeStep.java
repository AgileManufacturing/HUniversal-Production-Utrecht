package HAL.steps;

import java.io.Serializable;

import com.google.gson.JsonObject;

public class CompositeStep implements Serializable{
	private static final long serialVersionUID = 1206944727256435741L;
	private ProductStep productStep;
	private JsonObject command;
	
	public CompositeStep(ProductStep productStep, JsonObject command){
		this.command = command;
		this.productStep = productStep;
	}
	
	public ProductStep getProductStep(){
		return this.productStep;
	}
	public JsonObject getCommand(){
		return command;
	}
}
