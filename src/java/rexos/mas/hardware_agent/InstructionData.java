package rexos.mas.hardware_agent;

/**
 * @file rexos/mas/hardware_agent/InstructionData.java
 * @brief Provides a instructionData object
 * @date Created: 2013-04-24
 *
 * @author Thierry Gerritse
 * @author Hessel Meulenbeld
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright � 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

import rexos.mas.data.MongoSaveable;

import com.mongodb.BasicDBObject;

public class InstructionData implements MongoSaveable {
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
	
	public InstructionData() {}

	public String getCommand(){
		return command;
	}
	
	public String getDestination(){
		return destination;
	}
	
	public BasicDBObject getLookUpParameters(){
		return look_up_parameters;
	}
	
	public void setLookUpParameters(BasicDBObject look_up_parameters){
		this.look_up_parameters = look_up_parameters;
	}
	
	public BasicDBObject getPayload(){
		return payload;
	}
	
	public void setPayload(BasicDBObject payload){
		this.payload = payload;
	}
	
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("command", command);
		object.put("destination", destination);
		object.put("look_up", look_up);
		object.put("look_up_parameters", look_up_parameters);
		object.put("payload", payload);
		return object;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		
		command = object.getString("command");
		destination = object.getString("destination");
		look_up = object.getString("look_up");
		look_up_parameters = (BasicDBObject) object.get("look_up_parameters");
		payload = (BasicDBObject) object.get("payload");

	}

}
