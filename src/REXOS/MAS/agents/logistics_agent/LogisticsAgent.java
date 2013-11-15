/**
 * @file rexos/mas/logistics_agent/LogisticsAgent.java
 * @brief Agent charged with handling the logistics.
 * @date Created: 22 apr. 2013
 *
 * @author Peter Bonnema
 * @author Roy Scheefhals
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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
 * 
 **/
package agents.logistics_agent;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

import jade.core.Agent;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.Part;
import agents.data_classes.Position;
import agents.logistics_agent.behaviours.ArePartsAvailable;

/**
 * Agent charged with handling the logistics.
 * 
 */
public class LogisticsAgent extends Agent {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * Instantiates the agent and starts its behaviours.
	 * @see jade.core.Agent#setup()
	 */
	

	private Part supplyCratePart = new Part(2, 100, "GC4x4MB_1");
	private Part productCratePart = new Part(2, 101, "GC4x4MB_2");
	private Part whitePaper = new Part(3, 102, "WhitePaper");
	
	private HashMap<Part, Position> supplyCrateContent = new HashMap<Part, Position>();
	
	@Override
	public void setup() {
		Logger.log(LogLevel.NOTIFICATION, "" + this.getAID().getLocalName() + " spawned as a logistics agent.");
		addBehaviour(new ArePartsAvailable(this));
		
		//fill the crate
		for(int i = 0; i < 4; i++) {
			for(int j = 0; j < 4; j++) {
				supplyCrateContent.put(new Part(1, (i * 4) + j), new Position(j + 0.0, i + 0.0, supplyCratePart));
			}
		}
	}	
	
	public synchronized Entry<Part, Position> getBallPart(){
		Iterator<Entry<Part, Position>> it = supplyCrateContent.entrySet().iterator();
		if(it.hasNext()) {
			Entry<Part, Position> ball = it.next();
			supplyCrateContent.remove(ball.getKey());
			return ball;
		}
		return null;
	}
	
	public synchronized boolean isBallPartAvailable(){
		Iterator<Entry<Part, Position>> it = supplyCrateContent.entrySet().iterator();
		if(it.hasNext()) {
			return true;
		}
		return false;
	}
	
	public synchronized Part getSupplyCrate(){
		return supplyCratePart;
	}
	
	public synchronized Part getProductCrate(){
		return productCratePart;
	}
	
	public synchronized Part getWhitePaper(){
		return whitePaper;
	}
	
	/**
	 * 
	 * @see jade.core.Agent#takeDown()
	 * 
	 */
	@Override
	public void takeDown() {
		//
	}
}
