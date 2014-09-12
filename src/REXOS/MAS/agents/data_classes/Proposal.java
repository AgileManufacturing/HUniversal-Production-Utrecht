package MAS.agents.data_classes;

import jade.core.AID;

import java.io.Serializable;

import org.json.JSONException;
import org.json.JSONObject;

public class Proposal implements Serializable {
	private static final long serialVersionUID = 1067152669716744380L;
	private int startTime;
	private int duration;
	private int productStepId;
	private AID equipletAgent;
	
	public Proposal(JSONObject json, AID equipletAgent) throws JSONException{
		if (json.has("startTime")){
			this.startTime = json.getInt("startTime");
		}
		if (json.has("duration")){
			this.duration = json.getInt("duration");
		}
		if (json.has("productStepId")){
			this.productStepId = json.getInt("productStepId");
		}
		this.equipletAgent = equipletAgent;
	}

	public int getStartTime(){
		return this.startTime;
	}
	public int getDuration(){
		return this.duration;
	}
	public int getProductStepId(){
		return this.productStepId;
	}
	public AID getEquipletAgent(){
		return this.equipletAgent;
	}
}
