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

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import simulation.data.Capability;
import simulation.mas_entities.Product;
import simulation.mas_entities.ProductStep;

public class ProductSpawner implements Updatable {
	private static final String pathToProductCsvFile = "/home/t/sim/productA.csv";
	
	long nextSpawnTime;
	
	String productName;
	long productDeadline;
	ProductStep[] productSteps;
	int amountPerSpawn;
	double spawnInterval;
	
	Simulation simulation;
	
	public ProductSpawner(Simulation simulation) throws ParseException {
		this.simulation = simulation;
		
		String[][] fields = CSVReader.parseCsvFile(pathToProductCsvFile);
		
		productName = fields[0][0];
		productDeadline = Duration.parseDurationString(fields[1][0]);
		long start = Duration.parseDurationString(fields[2][0]);
		nextSpawnTime = simulation.getCurrentSimulationTime() + start;
		
		amountPerSpawn = Integer.parseInt(fields[3][0]);
		spawnInterval = Integer.parseInt(fields[4][0]);
		
		productSteps = new ProductStep[fields.length - 5];
		for(int i = 5; i < fields.length; i++) {
			productSteps[i - 5] = new ProductStep(Capability.getAvailableCapabilitiesById(Integer.parseInt(fields[i][0])));
		}		
	}
	
	@Override
	public void update(long time) {
		if(time >= nextSpawnTime) {
			Product[] products = spawnProducts();
			for(int i = 0; i < products.length; i++) {
				simulation.addUpdateable(products[i]);
			}
			
			nextSpawnTime += spawnInterval * 1000;
			System.out.println("Spawned products " + spawnInterval + " " + nextSpawnTime);
		}

	}
	private Product[] spawnProducts() {
		Product[] products = new Product[amountPerSpawn];
		for(int i = 0; i < products.length; i++) {
			products[i] = new Product(productSteps, simulation.getCurrentSimulationTime() + productDeadline);
		}
		return products; 
	}
}
