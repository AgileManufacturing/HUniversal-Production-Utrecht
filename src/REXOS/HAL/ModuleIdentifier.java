package HAL;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
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

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ModuleIdentifier other = (ModuleIdentifier) obj;
		if (manufacturer == null) {
			if (other.manufacturer != null)
				return false;
		} else if (!manufacturer.equals(other.manufacturer))
			return false;
		if (serialNumber == null) {
			if (other.serialNumber != null)
				return false;
		} else if (!serialNumber.equals(other.serialNumber))
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
	public String getSerialNumber(){
		return this.serialNumber;
	}
	
	public JSONObject toJSON(){
		JSONObject moduleIdentifier = new JSONObject();
		try {
			moduleIdentifier.putOpt("manufacturer", manufacturer);
			moduleIdentifier.putOpt("typeNumber", typeNumber);
			moduleIdentifier.putOpt("serialNumber", serialNumber);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return moduleIdentifier;
	}
	
	/*
	@Deprecated
	public boolean equals(ModuleIdentifier rhs) {
		if(this.manufacturer.equals(rhs.getManufacturer()) == false) return false;
		if(this.typeNumber.equals(rhs.getTypeNumber()) == false) return false;
		if(this.serialNumber.equals(rhs.getSerialNumber()) == false) return false;
		return true;
	}
	*/
	
	@Override
	public String toString() {
		return manufacturer + " " + typeNumber + " " + serialNumber;
	}
	
}