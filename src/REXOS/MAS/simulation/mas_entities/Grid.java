/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	Grid.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-17
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

package simulation.mas_entities;

import java.io.File;
import java.io.Reader;
import java.util.ArrayList;
import java.util.Date;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import agents.data_classes.Matrix;
import simulation.CSVReader;
import simulation.Simulation;
import simulation.Updatable;
import simulation.data.Capability;
import simulation.data.GridProperties;
import simulation.gui.MainGUI;


public class Grid implements Updatable{
	private static final double defaultDistance = 200.0;
	
	private Equiplet[][] equiplets;
	
	private Matrix distanceMatrix;
	
	private GridProperties gridProperties;
	
	public Grid(Equiplet[][] equiplets) {
		this.equiplets = equiplets;
	}
	public Grid(String equipletLayoutJsonFilePath, Simulation simulation) {
		
		JsonObject jsonObject = CSVReader.parseJsonObjectFile(MainGUI.gridFile.getAbsolutePath());
		JsonArray jsonEquipletsArray2D = jsonObject.get("grid").getAsJsonArray();
		
		gridProperties = new GridProperties(simulation, jsonObject.get("firstTimeSlot").getAsLong(), 
														   jsonObject.get("timeSlotLength").getAsLong(), 
														   jsonObject.get("equipletLoadWindow").getAsLong());
		
		equiplets = new Equiplet[jsonEquipletsArray2D.size()][];
		try {
			for(int i = 0; i < jsonEquipletsArray2D.size(); i++) {
				JsonArray jsonEquipletArray = jsonEquipletsArray2D.get(i).getAsJsonArray();
				equiplets[i] = new Equiplet[jsonEquipletArray.size()];
				
				for(int j = 0; j < jsonEquipletArray.size(); j++) {
					equiplets[i][j] = new Equiplet(jsonEquipletArray.get(j).getAsJsonObject(), gridProperties);
				}
			}
		} catch (NumberFormatException ex) {
			System.err.println("equipletLayoutCsv has an entry which could not be converted to int");
			throw ex;
		}
		
		initializeDistanceMatrix();
	}
	private void initializeDistanceMatrix() {
		distanceMatrix = new Matrix(equiplets.length * equiplets[0].length, equiplets.length * equiplets[0].length);
		for(int i = 0; i < distanceMatrix.getNumberOfRows(); i++) {
			int sourceEquipletY = i / equiplets.length; 
			int sourceEquipletX = i % equiplets.length;
			for(int j = 0; j < distanceMatrix.getNumberOfColumns(); j++) {
				int targetEquipletY = j / equiplets.length; 
				int targetEquipletX = j % equiplets.length;
				
				double distance = 
						Math.abs(targetEquipletY - sourceEquipletY) * defaultDistance + 
						Math.abs(targetEquipletX - sourceEquipletX) * defaultDistance;
				distanceMatrix.set(i, j, distance);
				
				//System.out.println(i + " " + j + " " + distance);
			}
		}
		System.out.println("Grid distanceMatrix:");
		distanceMatrix.show();
	}
	
	public Matrix getDistanceMatrix(){
		return distanceMatrix;
	}
	
	public double getDistanceBetweenEquiplets(Equiplet from, Equiplet to) {
		int fromIndex = -1;
		for(int i = 0; i < equiplets.length; i++) {
			for(int j = 0; j < equiplets[i].length; j++) {
				if(equiplets[i][j] == from) {
					fromIndex = i * equiplets[0].length + j;
					break;
				}
			}
		}
		int toIndex = -1;
		for(int i = 0; i < equiplets.length; i++) {
			for(int j = 0; j < equiplets[i].length; j++) {
				if(equiplets[i][j] == to) {
					toIndex = i * equiplets[0].length + j;
					break;
				}
			}
		}
		
		return distanceMatrix.get(fromIndex, toIndex);
	}
	
	@Override
	public void update(long time) {
		for(int i = 0; i < equiplets.length; i++) {
			for(int j = 0; j < equiplets[i].length; j++) {
				equiplets[i][j].update(time);
			}
		}
	}
	//In timeslots (PLEASE)
	public long GetMeanDistance() {
		return (long) (equiplets[0].length / 2 * defaultDistance + equiplets.length / 2 * defaultDistance); 
	}
	public Equiplet[] getEquiplets() {
		Equiplet[] output = new Equiplet[equiplets.length * equiplets[0].length];
		for(int i = 0; i < equiplets.length; i++) {
			for(int j = 0; j < equiplets[i].length; j++) {
				output[i * equiplets[0].length + j] = equiplets[i][j];
			}
		}
		return output;
	}
	public Equiplet[] getEquipletsWithoutReservation() {
		Equiplet[] allEquiplets = this.getEquiplets();
		ArrayList<Equiplet> equipletsWithoutReservation = new ArrayList<Equiplet>();
		for (Equiplet equiplet : allEquiplets) {
			if(equiplet.getBatchReservation().length == 0) equipletsWithoutReservation.add(equiplet);
		}
		return equipletsWithoutReservation.toArray(new Equiplet[equipletsWithoutReservation.size()]);
	}
	
	public Equiplet[] getEquipletsForReservation(int batchGroup) {
		Equiplet[] allEquiplets = this.getEquiplets();
		ArrayList<Equiplet> equipletsForReservation = new ArrayList<Equiplet>();
		for (Equiplet equiplet : allEquiplets) {
			Integer[] reservations = equiplet.getBatchReservation();
			for (Integer reservation : reservations) {
				if(reservation == batchGroup) equipletsForReservation.add(equiplet);
			}
		}
		Equiplet[] equipletsWithoutReservation = getEquipletsWithoutReservation();
		for (Equiplet equiplet : equipletsWithoutReservation) {
			equipletsForReservation.add(equiplet);
		}
		
		return equipletsForReservation.toArray(new Equiplet[equipletsForReservation.size()]);
	}
	
	public GridProperties getGridProperties(){
		return gridProperties;
	}
	
	public String toString() {
		String output = "Grid layout:\n";
		for(int i = 0; i < equiplets.length; i++) {
			for(int j = 0; j < equiplets[i].length; j++) {
				output += equiplets[i][j] + "\t";
			}
			output += "\n";
		}
		return output;
	}
}
