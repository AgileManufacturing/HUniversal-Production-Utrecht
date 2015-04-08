package HAL.dataTypes;

import java.io.Serializable;

import org.apache.commons.lang.builder.HashCodeBuilder;
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
public class ModuleIdentifier extends ModuleTypeIdentifier implements Serializable{
	private static final long serialVersionUID = -8091098772355637717L;
	
	public static final String SERIAL_NUMBER = "serialNumber";
	
	public String serialNumber;
	
	public ModuleIdentifier() {
		// nothing to do
	}
	public ModuleIdentifier(String manufacturer, String typeNumber, String serialNumber) {
		super(manufacturer, typeNumber);
		this.serialNumber = serialNumber;
	}


	public static ModuleIdentifier deSerialize(JSONObject input) throws JSONException {
		ModuleTypeIdentifier typeIdentifier = ModuleTypeIdentifier.deSerialize(input);
		ModuleIdentifier output = new ModuleIdentifier();
		output.manufacturer = typeIdentifier.manufacturer;
		output.typeNumber = typeIdentifier.typeNumber;
		output.serialNumber = input.getString(SERIAL_NUMBER);
		
		return output;
	}
	public JSONObject serialize() {
		JSONObject moduleIdentifier = super.serialize();
		try {
			moduleIdentifier.put(SERIAL_NUMBER, serialNumber);
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
		if ((obj instanceof ModuleIdentifier) == false)
			return false;
		ModuleIdentifier other = (ModuleIdentifier) obj;
		if (super.equals(obj) == false) {
			return false;
		}
		if (serialNumber == null) {
			if (other.serialNumber != null)
				return false;
		} else if (!serialNumber.equals(other.serialNumber))
			return false;
		return true;
	}
	@Override
	public int hashCode() {
		return new HashCodeBuilder().appendSuper(super.hashCode()).append(serialNumber).toHashCode();
	}
	@Override
	public String toString() {
		return super.toString() + " sNo " + serialNumber;
	}
}