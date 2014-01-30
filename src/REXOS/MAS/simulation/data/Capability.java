/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file src/REXOS/MAS/simulation/data/Capability.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	Dataclass for the representation of the capabilities that an equiplet can hold
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-17
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Roy Scheefhals
 *   .dMMMMMF           7Y=d9  dMMMMMr    	@author Tommas Bakker
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	@section LICENSE
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    	License:	newBSD
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		Copyright � 2013, HU University of Applied Sciences Utrecht. 
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  	All rights reserved.
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM     
 *                                        
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

package simulation.data;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.NoSuchElementException;

import simulation.CSVReader;


public class Capability {
	private int id;
	private String name;
	private int durationInTimeslots;
	
	public static Capability DummyCapability = new Capability("Dummy Capability", 1); 
	
	private static int idCounter = 0;
	private static HashMap<Integer, Capability> availableCapabilitiesById = new HashMap<Integer, Capability>();
	private static HashMap<String, Capability> availableCapabilitiesByName = new HashMap<String, Capability>();
	
	public static void loadCapabilities(String capabilitiesFilePath) {
		String[][] fields = CSVReader.parseCsvFile(capabilitiesFilePath);
		parseCapabilitiesCSV(fields);
	}
	
	/*
	public Capability(int id, String name, int durationInTimeSlots){
		this.id = id;
		this.name = name;
		this.durationInTimeslots = durationInTimeSlots;
	}*/
	
	public Capability(String name, int durationInTimeSlots) {
		id = idCounter++;
		this.name = name;
		this.durationInTimeslots = durationInTimeSlots;
	}

	public int getId() {
		return id;
	}

	public String getName() {
		return name;
	}
	
	public int getDuration() {
		return durationInTimeslots;
	}
	
	public String toCsvString() {
		return name + ", " + durationInTimeslots + "\r\n";
	}

	public static void parseCapabilitiesCSV(String[][] fields){
		try{
			for(int i = 0; i < fields.length; i++) {
				String capabiityName = fields[i][0];
				int capabilityDuration = Integer.parseInt(fields[i][1].trim());
				Capability capability = new Capability(capabiityName, capabilityDuration);
				
				availableCapabilitiesById.put(capability.getId(), capability);
				availableCapabilitiesByName.put(capabiityName, capability);
			}
		} catch(NumberFormatException e){
			System.err.println("Could not parse capabilties");
			e.printStackTrace();
		} catch(NoSuchElementException e1){
			System.err.println("wrong amount of elements when parsing capabilities");
			e1.printStackTrace();
		}
		printAvailableCapabilities();
	}
	
	public static Capability getCapabilityById(int id){
		Capability output = availableCapabilitiesById.get(id);
		if(output == null) throw new NullPointerException("No capability with id:" + id);
		return output;
	}
	public static Capability getCapabilityByName(String name){
		Capability output = availableCapabilitiesByName.get(name);
		if(output == null) throw new NullPointerException("No capability with name:" + name);
		return output;
	}
	public static Capability[] getCapabilities() {
		ArrayList<Capability> output = new ArrayList<Capability>(availableCapabilitiesById.size());
		for (Capability capability : availableCapabilitiesById.values()) {
			output.add(capability);
		}
		return output.toArray(new Capability[availableCapabilitiesById.size()]);
	}
	private static void printAvailableCapabilities(){
		Collection<Capability> values = availableCapabilitiesById.values();
		for ( Capability c1 : values){
			System.out.println("id: " + c1.getId() + ", name: " + c1.getName() + ", duration: " + c1.getDuration());
		}
	}
	
	public String toString() {
		return "id:" + id + " name:" + name + " durationInTimeslots: " + durationInTimeslots; 
	}
}
