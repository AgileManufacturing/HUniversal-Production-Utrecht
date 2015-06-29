package SCADA;

import org.json.JSONException;
import org.json.JSONObject;

import jade.core.AID;

/**
 * BasicAgentInfo stores information about an agent.
 * The GridAgent uses this class to reduce waiting times
 * and amount of messages needed for a GET_OVERVIEW message
 *
 */
public class BasicAgentInfo {
	
	private String type;
	private String state;
	private AID aid;
	
	/**
	 * Constructor of BasicAgentInfo takes the initial values 
	 * 
	 * @param aid AID of the agent
	 * @param state current state of the agent
	 * @param type type of the agent (for example EquipletAgent)
	 */
	public BasicAgentInfo(AID aid, String state, String type) {
		this.aid   = aid;
		this.state = state;
		this.type  = type;
	}
	/**
	 * Update the agent state value
	 * 
	 * @param state 
	 * @return
	 */
	public void updateState(String state) {
		this.state = state;
	}
	
	/**
	 * GetAID
	 * 
	 * @return the AID of the agent
	 */
	public AID getAID() {
		return this.aid;
	}
	
	/**
	 * GetType
	 * 
	 * @return the type of the agent
	 */
	public String getType() {
		return this.type;
	}
	
	/**
	 * GetState
	 * 
	 * @return the current state of the agent
	 */
	public String getState() {
		return this.state;
	}
	
	
	/**
	 * Get a JSONObject containing the id, type and state
	 * 
	 * @return the BasicAgentInfo in a JSONObject
	 */
	public JSONObject getJSONObject() {
		JSONObject o = new JSONObject();
		try {
			o.put("id", this.aid.getLocalName());
			o.put("type", this.type);
			o.put("state", this.state);
		} catch (JSONException e) {
			e.printStackTrace();
		}
		return o;
	}
}
