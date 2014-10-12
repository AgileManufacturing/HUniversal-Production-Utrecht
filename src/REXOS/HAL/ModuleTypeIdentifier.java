package HAL;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
/**
 * This class provides a method of identifying module types
 * @author Tommas Bakker
 *
 */
public class ModuleTypeIdentifier {
	private String manufacturer;
	private String typeNumber;
	
	public ModuleTypeIdentifier(String manufacturer, String typeNumber){
		this.manufacturer = manufacturer;
		this.typeNumber = typeNumber;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (obj instanceof ModuleTypeIdentifier)
			return false;
		ModuleTypeIdentifier other = (ModuleTypeIdentifier) obj;
		if (manufacturer == null) {
			if (other.manufacturer != null)
				return false;
		} else if (!manufacturer.equals(other.manufacturer))
			return false;
		if (typeNumber == null) {
			if (other.typeNumber != null)
				return false;
		} else if (!typeNumber.equals(other.typeNumber))
			return false;
		return true;
	}

	public String getManufacturer(){
		return this.manufacturer;
	}
	public String getTypeNumber(){
		return this.typeNumber;
	}
	public JSONObject toJSON(){
		JSONObject moduleIdentifier = new JSONObject();
		try {
			moduleIdentifier.putOpt("manufacturer", manufacturer);
			moduleIdentifier.putOpt("typeNumber", typeNumber);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return moduleIdentifier;
	}
	
	@Override
	public String toString() {
		return "man " + manufacturer + " tNr " + typeNumber;
	}
	
}