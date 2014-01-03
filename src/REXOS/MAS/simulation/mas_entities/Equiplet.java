/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	Equiplet.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-17
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Roy Scheefhals
 *   .dMMMMMF           7Y=d9  dMMMMMr    	@author	Alexander Streng
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

package simulation.mas_entities;

import java.util.ArrayList;
import java.util.Arrays;

import org.omg.CORBA.PRIVATE_MEMBER;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import simulation.Simulation;
import simulation.Updatable;
import simulation.data.Capability;
import simulation.data.EquipletError;
import simulation.data.GridProperties;
import simulation.data.ProductStep;
import simulation.data.ProductStepSchedule;
import simulation.data.TimeSlot;
import simulation.data.ProductStep.StepState;

public class Equiplet implements Updatable{
	
	public enum EquipletState{
		Idle,
		Error,
		Working
	}
	
	private EquipletState equipletState = EquipletState.Idle;
	
	private Capability[] capabilities;
	private String equipletName;
	private int reservedFor = 0;
	
	private EquipletError[] equipletErrors;
	
	private GridProperties gridProperties;
	
	private ArrayList<ProductStepSchedule> schedule = new ArrayList<ProductStepSchedule>();
	
	public Equiplet(JsonObject jsonArguments, GridProperties gridProperties){
		this.gridProperties = gridProperties;
		parseEquipletJson(jsonArguments);
	}
	
	/*public ArrayList<FreeTimeSlot> getFreeTimeSlots(){
		ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
		
		java.util.Collections.sort(schedule, new Comparator<ProductStepSchedule>() {
			@Override
			public int compare(ProductStepSchedule o1, ProductStepSchedule o2) {
				if (o1.getStartTimeSlot() < o2.getStartTimeSlot()){
					return -1;
				}
				else if(o1.getStartTimeSlot() == o2.getStartTimeSlot()){
					return 0;
				}
				else {
					return 1;
				}
			}
		});
		
		if (schedule.size() == 0 ) {
			//TODO: setup the CURRENT timeslot
			freeTimeSlots.add(new FreeTimeSlot(1l,null));
		}
	}*/
	
	public boolean canPerformStep(Capability capability){
		if (equipletState == EquipletState.Error){
			return false;
		}
		return Arrays.asList(capabilities).contains(capability);
	}
	
	public double getLoad() {
		return getLoad(new TimeSlot(
				TimeSlot.getCurrentTimeSlot(gridProperties.getSimulation(), gridProperties), 
				gridProperties.getTimeSlotLength()
		));
	}
	
	public double getLoad(TimeSlot timeSlot){
		
		long amountOfTimeSlotsBusy = 0;
		
		for (ProductStepSchedule productStepSchedule : schedule){
			if (productStepSchedule.getStartTimeSlot() + productStepSchedule.getDuration() > timeSlot.getStartTimeSlot() && 
					productStepSchedule.getStartTimeSlot() < (timeSlot.getStartTimeSlot() + gridProperties.getEquipletLoadWindow())){
				if(productStepSchedule.getStartTimeSlot() - timeSlot.getStartTimeSlot() < 0){
					amountOfTimeSlotsBusy +=  productStepSchedule.getDuration() - (productStepSchedule.getStartTimeSlot() - timeSlot.getStartTimeSlot());
				}
				if(productStepSchedule.getStartTimeSlot() + productStepSchedule.getDuration() - (timeSlot.getStartTimeSlot() + gridProperties.getEquipletLoadWindow())> 0){
					amountOfTimeSlotsBusy +=  productStepSchedule.getDuration() - (productStepSchedule.getStartTimeSlot() - timeSlot.getStartTimeSlot());
					break;
				}
				else{
					amountOfTimeSlotsBusy += productStepSchedule.getDuration();
				}
			}
		}
		if (amountOfTimeSlotsBusy > gridProperties.getEquipletLoadWindow() ) {
			System.out.println("The amount of count slots is higher than the window... wtf m8");
		}
		
		return amountOfTimeSlotsBusy / gridProperties.getEquipletLoadWindow();
	}
	
	public TimeSlot getFirstFreeTimeSlot(long startTimeSlot, long duration){
		
		//we want to have the first schedule available ... we expect here that the schedule 
		//is sorted from lowest starttimeslot to highest starttimeslot
		
		//nothing is scheduled so just give it back with indefinite duration
		if (schedule.size() == 0 ) {
			return new TimeSlot(startTimeSlot + 1, -1);
		}
		else{
			ProductStepSchedule curProductStepSchedule = schedule.get(0);
			
			//unit before the schedule
			if ((startTimeSlot + duration ) < curProductStepSchedule.getStartTimeSlot()){
				return new TimeSlot(startTimeSlot + 1, curProductStepSchedule.getStartTimeSlot() - startTimeSlot + 1);
			}
			ProductStepSchedule prevProductStepSchedule = curProductStepSchedule;
			
			//unit somewhere in between the schedule
			for (int iPlannedSteps = 1; iPlannedSteps < schedule.size() ; iPlannedSteps++){
				curProductStepSchedule = schedule.get(iPlannedSteps);
				
				if ( curProductStepSchedule.getStartTimeSlot() - (prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration()) >= duration ){
					return new TimeSlot(prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration(), 
							curProductStepSchedule.getStartTimeSlot() - (prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration()));
				}
			}
			
			//we have no other space then at the end of the schedule
			return new TimeSlot(schedule.get(schedule.size()-1).getStartTimeSlot() + schedule.get(schedule.size()-1).getDuration(), -1);
		}
	}
	
	public boolean isScheduleLocked(){
		return true;
	}
	
	public boolean schedule(ProductStep step, TimeSlot timeslot){
		//System.out.println("Equiplet: name " + equipletName + " scheduling " + step.toString());
		ProductStepSchedule newPSS= new ProductStepSchedule(step, timeslot);
		
		if (schedule.size() == 0 ){
			schedule.add(newPSS);
			return true;
		}
		
		//new step is at the first time
		if (newPSS.getStartTimeSlot() < schedule.get(0).getStartTimeSlot()){
			schedule.add(0,newPSS);
			return true;
		}
		
		// new step has to be somewhere in between the rest of the planned steps
		ProductStepSchedule curProductStepSchedule = schedule.get(0);
		ProductStepSchedule prevProductStepSchedule = curProductStepSchedule;
		
		for ( int iPlannedSteps = 1; iPlannedSteps < schedule.size(); iPlannedSteps++){
			curProductStepSchedule = schedule.get(iPlannedSteps);
			if (newPSS.getStartTimeSlot() < curProductStepSchedule.getStartTimeSlot() && 
					newPSS.getStartTimeSlot() > prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration() -1){
					schedule.add(iPlannedSteps, newPSS);
					return true;
			}
			prevProductStepSchedule = curProductStepSchedule;
		}
		
		//new schedule is after all the other scheduled steps
		if(newPSS.getStartTimeSlot() > schedule.get(schedule.size()-1).getStartTimeSlot()){
			schedule.add(newPSS);
			return true;
		}
		
		System.err.println("A step could not be added to the schedule, it does not fit anywhere in the schedule.");
		
		return false;
	}
	
	public void removeFromSchedule(ProductStep step){
		schedule.remove(step);
	}
	
	@Override
	public void update(long time) {
		
		EquipletError worstError = null;
		//check if an error has occurred and get the worst error ( the damaging type is the worst)
		for (EquipletError eqError : equipletErrors){
			if (eqError.isActive(time)){
				worstError = eqError;
				if (worstError.damagesProduct){
					break;
				}
			}
		}
		//process the error 
		if (worstError != null){
			if(equipletState != EquipletState.Error) {
				System.out.println("Equipet: " + equipletName + "has encountered error " + worstError);
			}
			ArrayList<Product> notifiedProducts = new ArrayList<Product>();
			if (equipletState == EquipletState.Working){
				ProductStep pStep = schedule.get(0).getProductStep();
				if (worstError.damagesProduct){
					pStep.setState(StepState.ProductError);
				}
				else{
					pStep.setState(StepState.ScheduleError);
					
				}
			}
			equipletState = EquipletState.Error;
		}
		else{
			//if there was an error that does not exist now, bring state to idle
			if (equipletState == EquipletState.Error){
				System.out.println("Equipet: " + equipletName + "has resolved its errors");
				equipletState = EquipletState.Idle;
			}
		}
		
		long currentTimeSlot = TimeSlot.getTimeSlotFromMillis(gridProperties, time);
		//update the schedule / simulate it is working
		if (schedule.size() > 0){
			if (equipletState == EquipletState.Working){
				ProductStepSchedule curProductStepSchedule = schedule.get(0);
				if (curProductStepSchedule.getStartTimeSlot() + curProductStepSchedule.getDuration() < currentTimeSlot){
					System.out.println("Equiplet: name " + equipletName + " has completed {" + 
							curProductStepSchedule.getProductStep().getCapability() + "} of product {" + 
							curProductStepSchedule.getProductStep().getProduct() + "}");
					//the step is done
					schedule.remove(0);
					equipletState = EquipletState.Idle;
					//notify the product object
					curProductStepSchedule.getProductStep().setFinished(true);
				}
			}
		}
		if (schedule.size() > 0){
			if (equipletState == EquipletState.Idle){
				ProductStepSchedule curProductStepSchedule = schedule.get(0);
				if ( currentTimeSlot == curProductStepSchedule.getStartTimeSlot()){
					equipletState = EquipletState.Working;
					//TODO: set the product step on working
				}
			}
		}
	}
	
	public EquipletState getEquipletState(){
		return equipletState;
	}
	
	@Override
	public String toString(){
		String result = "name: " + equipletName + ", capabilities: [";
		for (Capability cap : capabilities){
			result += cap.getName() + ",";
		}
		result += "], reservedFor: " + reservedFor;
		return result;
	}

	private void parseEquipletJson(JsonObject arguments){
		System.out.println("Parsing");
		equipletName = arguments.get("name").getAsString();
		
		JsonArray caps= arguments.get("capabilities").getAsJsonArray();
		capabilities = new Capability[caps.size()];
		for ( int iCaps = 0; iCaps < caps.size(); iCaps ++){
			capabilities[iCaps] =  Capability.getCapabilityByName(caps.get(iCaps).getAsString());
		}
		reservedFor = arguments.get("reservedFor").getAsInt();
		
		JsonArray errors = arguments.get("equipletErrors").getAsJsonArray();
		equipletErrors = new EquipletError[errors.size()];
		for (int iErrors = 0 ; iErrors < errors.size(); iErrors++){
			equipletErrors[iErrors] = new EquipletError(gridProperties.getSimulation(), errors.get(iErrors).getAsJsonObject());
		}
	}
}
