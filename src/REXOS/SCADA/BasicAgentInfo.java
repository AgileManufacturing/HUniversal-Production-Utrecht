package SCADA;

import jade.core.AID;

public class BasicAgentInfo {
	
	private AID aid;
	private String state;
	private String type;
	
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
	
	public String getState() {
		return this.state;
	}
	
	public String getType() {
		return this.type;
	}
}
