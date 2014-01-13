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
import simulation.data.Schedule;
import simulation.data.TimeSlot;
import simulation.data.ProductStep.StepState;

import org.apache.commons.collections4.list.TreeList;;


public class Equiplet implements Updatable{
	
	public enum EquipletState{
		Idle,
		Error,
		Working
	}
	private EquipletState equipletState = EquipletState.Idle;
	
	private Capability[] capabilities;
	private String equipletName;
	private ArrayList<Integer> reservedFor = new ArrayList<Integer>();
	
	private EquipletError[] equipletErrors;
	
	private GridProperties gridProperties;
	
	private TreeList<ProductStepSchedule> schedule = new TreeList<ProductStepSchedule>();
	
	private long timeSlotsActive = 0;
	private long timeSlotsIdle = 0;
	
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
		synchronized (schedule) {
			long amountOfTimeSlotsBusy = 0;
			
			for (ProductStepSchedule productStepSchedule : schedule){
				if (productStepSchedule.getStartTimeSlot() + productStepSchedule.getDuration() - 1 > timeSlot.getStartTimeSlot() && 
						productStepSchedule.getStartTimeSlot() < (timeSlot.getStartTimeSlot() + gridProperties.getEquipletLoadWindow() - 1)){
					
					// current product step starts before the start of the window
					if(productStepSchedule.getStartTimeSlot() < timeSlot.getStartTimeSlot()){
						//System.out.println("A" + (productStepSchedule.getDuration() - (timeSlot.getStartTimeSlot() - productStepSchedule.getStartTimeSlot())));
						amountOfTimeSlotsBusy += productStepSchedule.getDuration() - (timeSlot.getStartTimeSlot() - productStepSchedule.getStartTimeSlot());
						//amountOfTimeSlotsBusy +=  productStepSchedule.getDuration() - (productStepSchedule.getStartTimeSlot() - timeSlot.getStartTimeSlot());
					}
					// current product step ends after the end of the window
					else if(productStepSchedule.getStartTimeSlot() + productStepSchedule.getDuration() - 1 > (timeSlot.getStartTimeSlot() + gridProperties.getEquipletLoadWindow()) - 1){
						//System.out.println("B" + ((timeSlot.getStartTimeSlot() + gridProperties.getEquipletLoadWindow() - 1) - productStepSchedule.getStartTimeSlot()));
						amountOfTimeSlotsBusy += (timeSlot.getStartTimeSlot() + gridProperties.getEquipletLoadWindow() - 1) - productStepSchedule.getStartTimeSlot();
						//amountOfTimeSlotsBusy += productStepSchedule.getDuration() - (productStepSchedule.getStartTimeSlot() - timeSlot.getStartTimeSlot());
						break;
					}
					else{
						//System.out.println("C" + (productStepSchedule.getDuration()));
						amountOfTimeSlotsBusy += productStepSchedule.getDuration();
					}
				}
			}
			if (amountOfTimeSlotsBusy > gridProperties.getEquipletLoadWindow() ) {
				System.err.println("The amount of count slots is higher than the window... wtf m8");
			}
			
			return (double) amountOfTimeSlotsBusy / (double) gridProperties.getEquipletLoadWindow();
		}
	}
	
	public TimeSlot getFirstFreeTimeSlot(long startTimeSlot, long duration){
		synchronized (schedule) {
			//we want to have the first schedule available ... we expect here that the schedule 
			//is sorted from lowest starttimeslot to highest starttimeslot
			//System.out.println(equipletName + "getFree " + TimeSlot.getCurrentTimeSlot(gridProperties.getSimulation(), gridProperties) + " " + startTimeSlot + " " + duration);
			for (ProductStepSchedule s : schedule) {
	//			System.out.println("\t" + s.getStartTimeSlot() + "\t" + s.getDuration());
			}
			
			if(startTimeSlot == TimeSlot.getCurrentTimeSlot(gridProperties.getSimulation(), gridProperties)) {
				startTimeSlot++;
			}
			
			//nothing is scheduled so just give it back with indefinite duration
			if (schedule.size() == 0 ) {
				return new TimeSlot(startTimeSlot, -1);
			}
			else{
				ProductStepSchedule curProductStepSchedule = schedule.get(0);
				
				//unit before the schedule
				if ((startTimeSlot + duration ) < curProductStepSchedule.getStartTimeSlot()){
					return new TimeSlot(startTimeSlot, curProductStepSchedule.getStartTimeSlot() - startTimeSlot + 1);
				}
				ProductStepSchedule prevProductStepSchedule = curProductStepSchedule;
				
				//unit somewhere in between the schedule
				for (int iPlannedSteps = 1; iPlannedSteps < schedule.size() ; iPlannedSteps++){
					prevProductStepSchedule = schedule.get(iPlannedSteps - 1);
					curProductStepSchedule = schedule.get(iPlannedSteps);
					
					if ( prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration() - 1 >= startTimeSlot && 
	curProductStepSchedule.getStartTimeSlot() - (prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration()) >= duration ){
						//System.out.println("between");
						return new TimeSlot(prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration(), 
								curProductStepSchedule.getStartTimeSlot() - (prevProductStepSchedule.getStartTimeSlot() + prevProductStepSchedule.getDuration()));
					}
					
				}
				
				//we have no other space then at the end of the schedule
				//System.out.println("end");
				return new TimeSlot(schedule.get(schedule.size()-1).getStartTimeSlot() + schedule.get(schedule.size()-1).getDuration(), -1);
			}
		}
	}
	
	public boolean isScheduleLocked(){
		return true;
	}
	
	public boolean schedule(ProductStep step, TimeSlot timeslot){
		synchronized (schedule) {
			//System.out.println("Equiplet: name " + equipletName + " scheduling " + step.toString());
			//System.out.println(equipletName + "schedule " + TimeSlot.getCurrentTimeSlot(gridProperties.getSimulation(), gridProperties) + " " + timeslot.getStartTimeSlot() + " " + timeslot.getDuration());
			for (ProductStepSchedule s : schedule) {
	//			System.out.println("\t" + s.getStartTimeSlot() + "\t" + s.getDuration());
			}
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
				prevProductStepSchedule = schedule.get(iPlannedSteps - 1);
				
				if (newPSS.getStartTimeSlot() + newPSS.getDuration() - 1 < curProductStepSchedule.getStartTimeSlot() && 
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
			
			System.err.println("Equiplet " + equipletName + ": A step could not be added to the schedule, it does not fit anywhere in the schedule.");
			try {
				wait();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			return false;
		}
	}
	
	public void removeFromSchedule(ProductStep step){
		synchronized (schedule) {
			//System.out.print(equipletName + "is removing " + step);
			for (ProductStepSchedule pss : schedule) {
				if(pss.getProductStep() == step) { 
					boolean r = schedule.remove(pss);
					//System.out.println(r);
					break;
				}
			}
		}
	}
	
	@Override
	public void update(long time) {
		if(equipletState == EquipletState.Idle) {
			timeSlotsIdle++;
		} else if(equipletState == EquipletState.Working) {
			timeSlotsActive++;
		}
		
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
				//System.out.println("Equipet: " + equipletName + "has encountered error " + worstError);
			}
			if (equipletState == EquipletState.Working){
				ProductStep pStep = schedule.get(0).getProductStep();
				if (worstError.damagesProduct){
					//System.out.println(equipletName + "@1 setting " + schedule.get(0).getProductStep() + "to " + StepState.ProductError);
					pStep.setState(StepState.ProductError);
				} else{
					//System.out.println(equipletName + "@2 setting " + schedule.get(0).getProductStep() + "to " + StepState.ScheduleError);
					pStep.setState(StepState.ScheduleError);
				}
				// flush the remaining schedule
				for(int i = 1; i < schedule.size(); i++) {
					//System.out.println(equipletName + "@3 setting " + schedule.get(i).getProductStep() + "to " + StepState.ScheduleError);
					schedule.get(i).getProductStep().setState(StepState.ScheduleError);
				}
			} else {
				for(int i = 0; i < schedule.size(); i++) {
					//System.out.println(equipletName + "@4 setting " + schedule.get(i).getProductStep() + "to " + StepState.ScheduleError);
					schedule.get(i).getProductStep().setState(StepState.ScheduleError);
				}
			}
			equipletState = EquipletState.Error;
//			schedule.clear();
		}
		else{
			//if there was an error that does not exist now, bring state to idle
			if (equipletState == EquipletState.Error){
				//System.out.println("Equipet: " + equipletName + "has resolved its errors");
				equipletState = EquipletState.Idle;
			}
		}
		
		long currentTimeSlot = TimeSlot.getTimeSlotFromMillis(gridProperties, time);
		//update the schedule / simulate it is working
		if (schedule.size() > 0){
			if (equipletState == EquipletState.Working){
				ProductStepSchedule curProductStepSchedule = schedule.get(0);
				if (curProductStepSchedule.getStartTimeSlot() + curProductStepSchedule.getDuration() - 1 < currentTimeSlot){
					/*System.out.println("Equiplet: name " + equipletName + " has completed {" + 
							curProductStepSchedule.getProductStep().getCapability() + "} of product {" + 
							curProductStepSchedule.getProductStep().getProduct() + "}");*/
					//the step is done
					schedule.remove(0);
					equipletState = EquipletState.Idle;
					//notify the product object
					curProductStepSchedule.getProductStep().setState(StepState.Finished);;
				}
			}
		}
		if (schedule.size() > 0){
			if (equipletState == EquipletState.Idle){
				ProductStepSchedule curProductStepSchedule = schedule.get(0);
				if ( currentTimeSlot == curProductStepSchedule.getStartTimeSlot()){
					/*System.out.println("Equiplet: name " + equipletName + " is working on {" + 
							curProductStepSchedule.getProductStep().getCapability() + "} of product {" + 
							curProductStepSchedule.getProductStep().getProduct() + "}");*/
					equipletState = EquipletState.Working;
					curProductStepSchedule.getProductStep().setState(StepState.Working);
				} else if( currentTimeSlot > curProductStepSchedule.getStartTimeSlot()) {
					// this should not happen!
					System.err.println(equipletName + " Product step in schedule from the past: " + curProductStepSchedule.getProductStep() + "  " + curProductStepSchedule.getTimeSlot());
					System.err.println(equipletName + "schedule " + TimeSlot.getCurrentTimeSlot(gridProperties.getSimulation(), gridProperties));
					
					for (ProductStepSchedule s : schedule) {
						System.err.println("\t" + s.getStartTimeSlot() + "\t" + s.getDuration() + "\t" + s.getProductStep());
					}
					try {
						wait();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
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
		JsonArray reservedForArray = arguments.get("reservedFor").getAsJsonArray();
		for ( int i = 0; i < reservedForArray.size(); i++) {
			reservedFor.add(reservedForArray.getAsInt());
		}
		
		JsonArray errors = arguments.get("equipletErrors").getAsJsonArray();
		equipletErrors = new EquipletError[errors.size()];
		for (int iErrors = 0 ; iErrors < errors.size(); iErrors++){
			equipletErrors[iErrors] = new EquipletError(gridProperties.getSimulation(), errors.get(iErrors).getAsJsonObject());
		}
	}

	public String getName() {
		// TODO Auto-generated method stub
		return equipletName;
	}
	public Integer[] getBatchReservation() {
		return reservedFor.toArray(new Integer[reservedFor.size()]);
	}
	public TreeList<ProductStepSchedule> getSchedule() {
		return schedule;
	}
	public double getCurrentLoad() {
		return timeSlotsActive / (double)(timeSlotsActive + timeSlotsIdle);
	}
	public void resetCurrentLoad() {
		timeSlotsActive = 0;
		timeSlotsIdle = 0;
	}
}
