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
			output.mountPointX = input.getInt(MOUNT_POINT_X);
		}
		if(input.isNull(MOUNT_POINT_Y) == false) {
			output.mountPointY = input.getInt(MOUNT_POINT_Y);
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		if(attachedTo != null) {
			output.put(ATTACHED_TO, attachedTo.serialize());
		} else {
			output.put(ATTACHED_TO, JSONObject.NULL);
		}
		if(mountPointX != null) {
			output.put(MOUNT_POINT_X, mountPointX);
		} else {
			output.put(MOUNT_POINT_X, JSONObject.NULL);
		}
		if(mountPointY != null) {
			output.put(MOUNT_POINT_Y, mountPointY);
		} else {
			output.put(MOUNT_POINT_Y, JSONObject.NULL);
		}
		
		return output;
	}

}
