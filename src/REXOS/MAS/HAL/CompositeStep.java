package HAL;

import java.io.Serializable;

import com.google.gson.Gson;

public class CompositeStep implements Serializable{
	private static final long serialVersionUID = 1206944727256435741L;
	private ProductStep productStep;
	private Gson command;
	
	public CompositeStep(ProductStep productStep, Gson command){
		this.command = command;
		this.productStep = productStep;
	}
	
	public ProductStep getProductStep(){
		return this.productStep;
	}
	public Gson getCommand(){
		return command;
	}
}
