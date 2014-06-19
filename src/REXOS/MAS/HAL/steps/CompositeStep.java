package HAL.steps;

import generic.ProductStep;


import HAL.Capability;
import HAL.Module;
import HAL.exceptions.ModuleTranslatingException;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

/**
 * A CompositeStep is a step composed of multiple abstract {@link HardwareStep}s.
 * CompositeSteps are generated by {@link Capability} and interpeted by {@link Module}s.
 * @author Bas Voskuijlen
 *
 */
public class CompositeStep{
	private ProductStep productStep;
	private JsonObject command;
	private JsonObject relativeTo;
	
	public static final String IDENTIFIER = "identifier";
	public static final String LOOK_UP = "look_up";
	
	public CompositeStep(ProductStep productStep, JsonObject command, JsonObject relativeTo){
		this.command = command;
		this.relativeTo = relativeTo;
		this.productStep = productStep;
	}
	
	public ProductStep getProductStep(){
		return this.productStep;
	}
	public JsonObject getCommand(){
		return this.command;
	}
	public JsonObject getRelativeTo(){
		return this.relativeTo;
	}
	public JsonObject popCommandIdentifier(String identifier) throws ModuleTranslatingException{
		JsonElement jsonIdentifier = command.remove(identifier);
		if (jsonIdentifier == null){
			throw new ModuleTranslatingException ("Module didn't find a \"" + identifier + "\" key in CompositeStep command: " + command, this);
		}		
		return jsonIdentifier.getAsJsonObject();
	}
}
