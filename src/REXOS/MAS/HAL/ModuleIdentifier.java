package HAL;

import com.google.gson.JsonObject;
/**
 * This class provides a method of identifying modules
 * @author Tommas Bakker
 *
 */
public class ModuleIdentifier {
	private String manufacturer;
	private String typeNumber;
	private String serialNumber;
	
	public ModuleIdentifier(String manufacturer, String typeNumber, String serialNumber){
		this.manufacturer = manufacturer;
		this.typeNumber = typeNumber;
		this.serialNumber = serialNumber;
	}

	public String getManufacturer(){
		return this.manufacturer;
	}
	public String getTypeNumber(){
		return this.typeNumber;
	}
	public String getSerialNumber(){
		return this.serialNumber;
	}
	
	public JsonObject getAsJSON(){
		JsonObject moduleIdentifier = new JsonObject();
		moduleIdentifier.addProperty("manufacturer", manufacturer);
		moduleIdentifier.addProperty("typeNumber", typeNumber);
		moduleIdentifier.addProperty("serialNumber", serialNumber);
		return moduleIdentifier;
	}
}