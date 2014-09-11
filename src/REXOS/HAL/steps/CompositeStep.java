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
	private OriginPlacement originPlacement;
	
	public static final String IDENTIFIER = "identifier";
	public static final String RELATIVE_TO = "relativeTo";
	
	public CompositeStep(ProductStep productStep, JsonObject command, OriginPlacement originPlacement){
		this.command = command;
		this.originPlacement = originPlacement;
		this.productStep = productStep;
	}
	
	public ProductStep getProductStep(){
		return this.productStep;
	}
	public JsonObject getCommand(){
		return this.command;
	}
	public OriginPlacement getOriginPlacement(){
		return this.originPlacement;
	}
	public JsonElement popCommandIdentifier(String identifier) throws ModuleTranslatingException{
		JsonElement jsonIdentifier = command.remove(identifier);
		
		if (jsonIdentifier == null){
			throw new ModuleTranslatingException ("Module didn't find a \"" + identifier + "\" key in CompositeStep command: " + command, this);
		}
		return jsonIdentifier;
	}
	
	public JsonObject toJSON() {
		JsonObject returnValue = new JsonObject();
		returnValue.add("command", command);
		returnValue.add("originPlacement", originPlacement.toJSON());
		return returnValue;
	}
}
