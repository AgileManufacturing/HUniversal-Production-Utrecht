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
	
	public JsonObject toJSON(){
		JsonObject moduleIdentifier = new JsonObject();
		moduleIdentifier.addProperty("manufacturer", manufacturer);
		moduleIdentifier.addProperty("typeNumber", typeNumber);
		moduleIdentifier.addProperty("serialNumber", serialNumber);
		return moduleIdentifier;
	}
	
	public boolean equals(ModuleIdentifier rhs) {
		if(this.manufacturer.equals(rhs.getManufacturer()) == false) return false;
		if(this.typeNumber.equals(rhs.getTypeNumber()) == false) return false;
		if(this.serialNumber.equals(rhs.getSerialNumber()) == false) return false;
		return true;
	}
}