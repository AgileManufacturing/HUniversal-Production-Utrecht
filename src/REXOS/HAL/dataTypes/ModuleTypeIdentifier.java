package HAL.dataTypes;

import java.io.Serializable;

import org.apache.commons.lang.builder.HashCodeBuilder;
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
public class ModuleTypeIdentifier implements Serializable{
	private static final long serialVersionUID = -6860907911597941990L;
	
	public static final String MANUFACTURER = "manufacturer";
	public static final String TYPE_NUMBER = "typeNumber";
	
	public String manufacturer;
	public String typeNumber;
	
	public ModuleTypeIdentifier() {
		// nothing to do
	}
	public ModuleTypeIdentifier(String manufacturer, String typeNumber){
		this.manufacturer = manufacturer;
		this.typeNumber = typeNumber;
	}
	
	public static ModuleTypeIdentifier deSerialize(JSONObject input) throws JSONException {
		ModuleTypeIdentifier output = new ModuleTypeIdentifier();
		
		output.manufacturer = input.getString(MANUFACTURER);
		output.typeNumber = input.getString(TYPE_NUMBER);
		
		return output;
	}
	public JSONObject serialize(){
		JSONObject moduleIdentifier = new JSONObject();
		try {
			moduleIdentifier.put(MANUFACTURER, manufacturer);
			moduleIdentifier.put(TYPE_NUMBER, typeNumber);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return moduleIdentifier;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if ((obj instanceof ModuleTypeIdentifier) == false)
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
	@Override
	public int hashCode() {
		return new HashCodeBuilder().append(manufacturer).append(typeNumber).toHashCode();
	}
	@Override
	public String toString() {
		return "man " + manufacturer + " tNo " + typeNumber;
	}
	
}