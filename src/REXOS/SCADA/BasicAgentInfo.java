package SCADA;

import org.json.JSONException;
import org.json.JSONObject;

import jade.core.AID;

public class BasicAgentInfo {
	
	private String type;
	private String state;
	private AID aid;
	
	public BasicAgentInfo(AID aid, String state, String type) {
		this.aid   = aid;
		this.state = state;
		this.type  = type;
	}
	
	public void updateState(String state) {
		this.state = state;
	}
	
	public AID getAID() {
		return this.aid;
	}
	
	public String getType() {
		return this.type;
	}
	
	public String getState() {
		return this.state;
	}
	
	public JSONObject getJSONObject() {
		JSONObject o = new JSONObject();
		try {
			o.put("id", this.aid.getLocalName());
			o.put("type", this.type);
			o.put("state", this.state);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return o;
	}
}
