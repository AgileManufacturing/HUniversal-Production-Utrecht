/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	ProductSpawner.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-18
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Tommas Bakker
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright � 2013, HU University of Applied Sciences Utrecht. 
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  
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

package simulation;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;

import simulation.data.Capability;
import simulation.data.ProductStep;
import simulation.data.Schedule;
import simulation.data.TimeSlot;
import simulation.gui.MainGUI;
import simulation.mas_entities.Equiplet;
import simulation.mas_entities.Grid;
import simulation.mas_entities.Product;

public class DynamicProductSpawner implements Updatable {
	long prevTime;
	
	String productName;
	double targetLoad;
	int seed;
	int minProductLength;
	int maxProductLength;
	double chance;
	HashMap<Capability, Integer> workForce = new HashMap<Capability, Integer>();
	HashMap<Capability, Double> capabilityProgress = new HashMap<Capability, Double>();
	public long totalDurationInTimeSlots = 0;
	
	
	Simulation simulation;
	Grid grid;
	Random random;
	
	public DynamicProductSpawner(Simulation simulation, Grid grid) throws ParseException {
		this.simulation = simulation;
		this.grid = grid;
		
		String[][] fields = CSVReader.parseCsvFile(MainGUI.dynamicProductsFile.getAbsolutePath());
		
		productName = fields[0][0];
		long start = Duration.parseDurationString(fields[1][0]);
		prevTime = simulation.getCurrentSimulationTime() + start;
		seed = Integer.parseInt(fields[2][0]);
		minProductLength = Integer.parseInt(fields[3][0]);
		maxProductLength = Integer.parseInt(fields[4][0]);
		targetLoad = Double.parseDouble(fields[5][0]);
		chance = Double.parseDouble(fields[6][0]);
		random = new Random(seed);
		
		Capability[] capabilities = Capability.getCapabilities();
		for (Capability capability : capabilities) {
			int workForceForCurrentCapability = 0;
			Equiplet[] equiplets = grid.getEquiplets();
			for (Equiplet equiplet : equiplets) {
				if(equiplet.canPerformStep(capability) == true) {
					workForceForCurrentCapability++;
				}
			}
			workForce.put(capability, workForceForCurrentCapability);
			capabilityProgress.put(capability, 0.0);
			//System.out.println(capability + " : " + workForceForCurrentCapability);
		}
	}
	
	@Override
	public void update(long time) {
		double timeSlotsElapsed = (time - prevTime) / (double) grid.getGridProperties().getTimeSlotLength();
		
		boolean needToSpawnProducts = false;
		for (Capability capability : capabilityProgress.keySet()) {
			double newValue = capabilityProgress.get(capability) + timeSlotsElapsed / (double)capability.getDuration() * workForce.get(capability) * targetLoad;
			capabilityProgress.put(capability, newValue);
			if(newValue >= 1) needToSpawnProducts = true;
		}
		if(needToSpawnProducts == true && random.nextDouble() < chance) {
			Product[] products = spawnProducts();
			for(int i = 0; i < products.length; i++) {
				
				simulation.addUpdateable(products[i]);
			}
			//System.out.println("DynamicProductSpawner: Spawned " + products.length + " " + productName + "s:");
		/*	for (Product product : products) {
				//System.out.println("\t" + product);
				for (ProductStep step : product.getProductSteps()) {
					//System.out.println("\t\t" + step);
				}
			} */
		}
		prevTime = time;
	}
	
	private Product[] spawnProducts() {
		ArrayList<Capability> capabilitiesToPerform = new ArrayList<Capability>();
		for (Capability capability : capabilityProgress.keySet()) {
			while(capabilityProgress.get(capability) >= 1.0) {
				capabilitiesToPerform.add(capability);
				capabilityProgress.put(capability, capabilityProgress.get(capability) - 1.0);
			}
		}
		ArrayList<Product> products = new ArrayList<Product>();
		while(capabilitiesToPerform.size() != 0) {
			int stepsCount = random.nextInt(maxProductLength - minProductLength + 1) + minProductLength;
			ArrayList<Capability> productCapabilities = new ArrayList<Capability>();
			//System.out.println(stepsCount + "  " + capabilitiesToPerform.size());
			for(int i = 0; i < stepsCount; i++) {
				if(capabilitiesToPerform.size() != 0) {
					int index = random.nextInt(capabilitiesToPerform.size());
					//System.out.println(capabilitiesToPerform.get(index));
					productCapabilities.add(capabilitiesToPerform.get(index));
					totalDurationInTimeSlots += capabilitiesToPerform.get(index).getDuration();
					capabilitiesToPerform.remove(index);
				}
			}
			products.add(new Product(productName, simulation, grid, productCapabilities.toArray(new Capability[productCapabilities.size()]), simulation.getCurrentSimulationTime() + 86 * 1000));
		}
		return products.toArray(new Product[products.size()]);
	}
}
