package rexos.mas.hardware_agent;

import com.mongodb.BasicDBObject;

import rexos.mas.data.IMongoSaveable;

public class InstructionData implements IMongoSaveable {
	private String command;
	private String destination;
	private String look_up;
	private BasicDBObject look_up_parameters;
	private BasicDBObject payload;
	
	public InstructionData(String command, String destination,String look_up,
			BasicDBObject look_up_parameters,BasicDBObject payload){
		
		this.command = command;
		this.destination = destination;
		this.look_up = look_up;
		this.look_up_parameters = look_up_parameters;
		this.payload = payload;
		
	}
	
	public InstructionData(BasicDBObject object){
		
		fromBasicDBObject(object);
		
	}
	
	public InstructionData() {
		// TODO Auto-generated constructor stub
	}

	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("command", command);
		object.put("command", destination);
		object.put("look_up", look_up);
		object.put("look_up_parameters", look_up_parameters);
		object.put("payload", payload);
		return object;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		
		command = object.getString("command");
		command = object.getString("command");
		look_up = object.getString("look_up");
		look_up_parameters = (BasicDBObject) object.get("look_up_parameters");
		payload = (BasicDBObject) object.get("payload");

	}

}
