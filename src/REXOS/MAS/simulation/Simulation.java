/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file src/REXOS/MAS/simulation/Simulation.java
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

import simulation.collectors.DataCollector;
import simulation.data.TimeSlot;
import simulation.mas_entities.Grid;

import com.sun.xml.internal.bind.v2.runtime.unmarshaller.XsiNilLoader.Array;

public class Simulation implements Runnable{
	/** 
	 * boolean indicating whether the simulation is currently running 
	 */
	private boolean isRunning = false;
	/**
	 * boolean indicating whether the simulation has been aborted. The simulation can be aborted by calling abort()
	 */
	private boolean isAborted = false;
	/**
	 * boolean indicating whether the simulation has finished successfully.
	 */
	private boolean isFinished = false;
	/**
	 * The interval in seconds between ticks. 
	 * Normally set to 0 (for maximum simulation speed). Adjust when running a realtime simulation.
	 */
	private double interval = 0.00; // seconds
	private double tickTime = 0.1; // seconds
	private double duration = 2 * 60 * 60 * 1000 + 0 * 60 * 1000; // milliseconds
	
	/**
	 * The current tick number of the simulation. Starts at 0
	 */
	private long tick = 0;
	/**
	 * The system time in milliseconds at the moment that the simulation started
	 */
	private long startSimulationTime;
	/**
	 * The current simulation time in milliseconds. Should be equal to startSimulationTime + tick * tickTime
	 */
	private long currentSimulationTime;
	/**
	 * A list of all the updatables that should be updated every tick
	 */
	private ArrayList<Updatable> updatables = new ArrayList<Updatable>();
	/**
	 * A list of all the updatables that should be added. The simulation adds these updatables at the end of each tick. 
	 */
	private ArrayList<Updatable> updatablesToBeAdded = new ArrayList<Updatable>();
	/**
	 * A list of all the updatables that should be removed. The simulation adds these updatables at the end of each tick.
	 */
	private ArrayList<Updatable> updatablesToBeRemoved = new ArrayList<Updatable>();
	/**
	 * A list of all the datacollectors that should be updated
	 */
	private ArrayList<DataCollector> dataCollectors = new ArrayList<DataCollector>();
	/**
	 * A list of all the datacollectors that should be added. The simulation adds these datacollectors at the end of each tick. 
	 */
	private ArrayList<DataCollector> dataCollectorsToBeAdded = new ArrayList<DataCollector>();
	/**
	 * The thread used to run the simulation
	 */
	private Thread thread;
	
	/**
	 * The grid for this simulation. Should be removed?
	 */
	public Grid grid;
	
	/**
	 * Pauses the simulation. The simulation will finish the current tick and will halt before executing the next tick.
	 */
	public void pauseSimulation(){
		isRunning = false;
	}
	/**
	 * Resumes the simulation.
	 */
	public synchronized void resumeSimulation() {
		isRunning = true;
		notify();
	}
	/**
	 * Aborts the simulation
	 */
	public synchronized void abort() {
		isAborted = true;
	}
	/**
	 * This method will wait until the simulation thread has died (which happens when the simulation is either aborted of finished).
	 * @throws Exception if the simulation is currently paused.
	 */
	public void waitUntilFinished() throws Exception{
		if(isRunning == false) throw new Exception("Pointsless to wait on a simulation which is not running");
		thread.join();
	}
	/**
	 * Constructs the simulation and starts the thread but does NOT start the simulation
	 */
	public Simulation() {
		startSimulationTime = System.currentTimeMillis();
		currentSimulationTime = startSimulationTime;
		thread = new Thread(this);
		thread.setPriority(Thread.MIN_PRIORITY + 1);
		thread.start();
	}
	
	public synchronized long getCurrentSimulationTime() {
		return currentSimulationTime;
	}
	
	public synchronized long getStartSimulationTime() {
		return startSimulationTime;
	}
	/**
	 * Add an updateable to the simulation. The updateable will be added at the end of the tick
	 * @param updateable
	 */
	public synchronized void addUpdateable(Updatable updateable) {
		updatablesToBeAdded.add(updateable);
	}
	
	/**
	 * Add an dataCollector to the simulation. The dataCollector will be added at the end of the tick
	 * @param dataCollector
	 */
	public synchronized void addDataCollector(DataCollector dataCollector) {
		dataCollectorsToBeAdded.add(dataCollector);
	}
	
	/**
	 * Get the progress of the simulation
	 * @return the progress with [0.0, 1.0]
	 */
	public synchronized double getProgress(){
		return (currentSimulationTime - startSimulationTime) / duration;
	}
	
	/**
	 * The method for the simulation thread. Do NOT call!
	 */
	public void run(){
		while(isAborted == false) {
			synchronized (this) {
				long cycleStartTime = System.currentTimeMillis();
				if(isRunning == false) {
					try {
						System.out.println("Simulation halted on tick " + tick);
						wait();
					} catch (InterruptedException e) {
						// ignore, this is expected
					}
				}
				
				/*System.out.println("--- Simulation: turn " + turn + " time " + 
						currentSimulationTime + " (" + (currentSimulationTime - startSimulationTime) + ")" + 
						"timeSlot " + TimeSlot.getCurrentTimeSlot(this, grid.getGridProperties()));*/
				
				// update simulation
				for (Updatable updateable : updatables) {
					updateable.update(currentSimulationTime);
				}
				
				// gather information
				for (DataCollector dataCollector : dataCollectors) {
					dataCollector.collectData(currentSimulationTime);
				}
				
				// update the time
				currentSimulationTime += tickTime * 1000;
				tick++;
				
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
				
				while(updatablesToBeAdded.size() != 0) {
					updatables.add(updatablesToBeAdded.remove(0));
				}
				while(updatablesToBeRemoved.size() != 0) {
					updatables.remove(updatablesToBeRemoved.remove(0));
				}
				
				
				while(dataCollectorsToBeAdded.size() != 0) {
					dataCollectors.add(dataCollectorsToBeAdded.remove(0));
				}
				
				// are we finished?
				if(currentSimulationTime - startSimulationTime > duration) {
					isFinished = true;
					break;
				}
			}
			Thread.yield();
		}
		// exit simulation thread
	}
	
	/**
	 * Get the updatables currently in the simulation (without the updatables to be added and with the updatables to be removed)
	 * @return an array of updatables
	 */
	public Updatable[] getUpdatables() {
		return updatables.toArray(new Updatable[updatables.size()]);
	}
	/**
	 * Temoves the updatable to the list of updatables by first adding it to the list of updatablesToBeRemoved
	 * @param updateable
	 */
	public void removeUpdateable(Updatable updateable) {
		updatablesToBeRemoved.add(updateable);
	}
	/**
	 * returns true if the simulation is finished
	 * @return
	 */
	public boolean getIsFinished() {
		return isFinished;
	}
}
