package HAL;

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
public class ModuleIdentifier extends ModuleTypeIdentifier{
	private String serialNumber;
	
	public ModuleIdentifier(String manufacturer, String typeNumber, String serialNumber){
		super(manufacturer, typeNumber);
		this.serialNumber = serialNumber;
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

	public String getSerialNumber(){
		return this.serialNumber;
	}
	
	public JSONObject toJSON(){
		JSONObject moduleIdentifier = super.toJSON();
		try {
			moduleIdentifier.putOpt("serialNumber", serialNumber);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return moduleIdentifier;
	}
	
	@Override
	public String toString() {
		return super.toString() + " sNr " + serialNumber;
	}
}