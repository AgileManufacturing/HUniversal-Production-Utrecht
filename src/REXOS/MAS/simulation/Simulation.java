/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	Simulation.java
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

package simulation;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

public class Simulation implements Runnable{
	
	private boolean isRunning = false;
	private double interval = 0.0; // seconds
	private double turnTime = 0.01; // seconds
	
	private long turn = 0;
	private long startSimulationTime;
	private long currentSimulationTime;
	private ArrayList<Updatable> updateables = new ArrayList<Updatable>();
	private ArrayList<Updatable> updateablesToBeAdded = new ArrayList<Updatable>();
	private Thread thread;
	
	public void pauseSimulation(){
		isRunning = false;
	}
	public synchronized void resumeSimulation(){
		isRunning = true;
		notify();
	}
	
	public Simulation() {
		startSimulationTime = System.currentTimeMillis();
		
		thread = new Thread(this);
		thread.start();
	}
	
	public long getCurrentSimulationTime() {
		return currentSimulationTime;
	}
	
	public long getStartSimulationTime() {
		return startSimulationTime;
	}
	
	public void addUpdateable(Updatable updateable) {
		updateablesToBeAdded.add(updateable);
	}
	
	public void run(){
		while(true) {
			synchronized (this) {
				long cycleStartTime = System.currentTimeMillis();
				if(isRunning == false) {
					try {
						System.out.println("Simulation halted on turn " + turn);
						wait();
					} catch (InterruptedException e) {
						// ignore, this is expected
					}
				}
				
				// update the time
				currentSimulationTime += turnTime * 1000;
				
				//System.out.println("--- Simulation: turn " + turn + " time " + currentSimulationTime);
				
				// update simulation
				for (Updatable updateable : updateables) {
					updateable.update(currentSimulationTime);
				}
				
				turn++;
				// sleep if dealing with 'realtime' simulation
				long cycleEndTime = System.currentTimeMillis();
				long cycleDuration = cycleEndTime - cycleStartTime;
				if(cycleDuration < (interval * 1000)) {
					try {
						Thread.sleep((long) ((interval * 1000) - cycleDuration));
					} catch (InterruptedException e) {
						// ignore
					}
				} else {
					// no need to sleep at all
				}
				updateables.addAll(updateablesToBeAdded);
			}
		}
		// exit simulation thread
	}
}
