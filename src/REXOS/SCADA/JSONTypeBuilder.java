package SCADA;

import org.json.JSONException;
import org.json.JSONObject;

public class JSONTypeBuilder {

	public JSONObject getIntObject(int value, boolean readOnly, boolean required){
		JSONObject object = new JSONObject();
		try {
			object.put("type", "int");
			object.put("value", value);
			object.put("read-only", readOnly);
			object.put("required", required);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return object;
	}
	
	public JSONObject getStringObject(String value, boolean readOnly, boolean required){
		JSONObject object = new JSONObject();
		try {
			object.put("type", "string");
			object.put("value", value);
			object.put("read-only", readOnly);
			object.put("required", required);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return object;
	}
	
}
