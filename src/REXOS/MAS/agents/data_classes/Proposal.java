package MAS.agents.data_classes;

import jade.core.AID;

import java.io.Serializable;

import com.google.gson.JsonObject;

public class Proposal implements Serializable {
	private static final long serialVersionUID = 1067152669716744380L;
	private int startTime;
	private int duration;
	private int productStepId;
	private AID equipletAgent;
	
	public Proposal(JsonObject json, AID equipletAgent){
		if (json.has("startTime")){
			this.startTime = json.get("startTime").getAsInt();
		}
		if (json.has("duration")){
			this.duration = json.get("duration").getAsInt();
		}
		if (json.has("productStepId")){
			this.productStepId = json.get("productStepId").getAsInt();
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
