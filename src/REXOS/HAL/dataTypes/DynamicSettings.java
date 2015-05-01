package HAL.dataTypes;

import org.json.JSONException;
import org.json.JSONObject;

public class DynamicSettings {
	public static final String ATTACHED_TO = "attachedTo";
	public static final String MOUNT_POINT_X = "mountPointX";
	public static final String MOUNT_POINT_Y = "mountPointY";
	
	public ModuleIdentifier attachedTo;
	public Integer mountPointX, mountPointY;
	public Integer rotation;
	
	public static DynamicSettings deSerialize(JSONObject input) throws JSONException {
		DynamicSettings output = new DynamicSettings();
		
		if(input.isNull(ATTACHED_TO) == false) {
			output.attachedTo = ModuleIdentifier.deSerialize(input.getJSONObject(ATTACHED_TO));
		}
		if(input.isNull(MOUNT_POINT_X) == false) {
			output.mountPointX = Integer.getInteger(input.get(MOUNT_POINT_X).toString());
		}
		if(input.isNull(MOUNT_POINT_Y) == false) {
			output.mountPointY = Integer.getInteger(input.get(MOUNT_POINT_Y).toString());
		}
		
		return output;
	}

}
