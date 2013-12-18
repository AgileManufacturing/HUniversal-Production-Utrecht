/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	Product.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-17
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Alexander Streng
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

import java.util.ArrayList;
import java.util.LinkedHashMap;

import simulation.data.Schedule;
import simulation.data.TimeSlot;
import agents.data_classes.Matrix;

public class Product {
	
	private ProductStep[] productSteps;
	private Equiplet[] equiplets;
	private long deadline;
	private LinkedHashMap<ProductStep, Schedule> finalSchedules; //We might want to keep the order of the list.
	private Grid grid;
	
	public Product(Grid grid, ProductStep[] productSteps, long deadline){
		this.productSteps = productSteps;
		this.deadline = deadline;
		this.grid = grid;
		finalSchedules = new LinkedHashMap<ProductStep, Schedule>();
		
		long currentTimeSlot = TimeSlot.getCurrentTimeSlot();
		//We need to pass the current timeslot, to prevent synchronisation issues.
		schedule(currentTimeSlot, generateScheduleMatrix(equiplets, productSteps, currentTimeSlot));
	}

	/**
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more information see 
	 * @param currentTimeSlot 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user requirements to product' - Leo van Moergestel section 3.2 
	 * @return the generated scheduleMatrix
	 */
	private Matrix generateScheduleMatrix(Equiplet[] equiplets, ProductStep[] productSteps, long currentTimeSlot) {		
		//construct a new matrices to perform a neat-o selection
		Matrix scheduleMatrix = new Matrix(equiplets.length, productSteps.length);
		
		int sequenceLength, firstInSequence;
		
			//Iterate through them steps to fill the matrix
			for (int row = 0; row < equiplets.length; row++) { // row ( equiplets )
				sequenceLength = 0; firstInSequence = -1; // always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
				
				long scheduleTimeSlot = currentTimeSlot; // scheduletimeslot has to be the same for each equiplet.
				
				for (int column = 0; column < equiplets.length; column++) { // column ( product steps
					
					double canPerformStepValue = equiplets[row].canPerformStep(productSteps[column].getCapability()) ? 1.0 : 0.0;
				
					if(canPerformStepValue == 1.0) {   //increase sequence counter.
						if(firstInSequence < 0){	  //set the first item in the sequence.
							firstInSequence = column;
						}
						sequenceLength++;
						if(column == productSteps.length){ // end of row
							setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
						}
					} else if(canPerformStepValue == 0.0 && sequenceLength > 0) { // end of sequence
						setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
						sequenceLength = 0;
						firstInSequence = -1;
					}
					
					//value might have changed since we added sequence multiplier
					double loadValue = equiplets[row].getLoad(equiplets[row].getFirstFreeTimeSlot(scheduleTimeSlot, productSteps[column].getDuration()));
					
					//Multiply with load value ( e.g. the load of the equiplet )
					scheduleMatrix.set(row, column, (scheduleMatrix.get(row, column) * loadValue));
					
					//we still need a transportstep?
					scheduleTimeSlot += grid.getDistanceMatrix().get(1, 1);
					//add the time to the scheduleTimeSlot
					scheduleTimeSlot += productSteps[column].getCapability().getDuration();
				}
		}
		return scheduleMatrix;
	}

	private void schedule(long currentTimeSlot, Matrix scheduleMatrix) {
		// Read the matrix. Write function to iterate each seperate row ( productsteps ) and pick each equiplet 		
		for (int column = 0; column < scheduleMatrix.getNumberOfColumns(); column++) { //Productsteps 
			
			int highestEquipletScoreIndex = -1;
			ProductStep productStep = productSteps[column];
			
			for (int row = 0; row < scheduleMatrix.getNumberOfRows(); row++) { //AID'S
				highestEquipletScoreIndex = (scheduleMatrix.get(row, column) > scheduleMatrix.get(highestEquipletScoreIndex, column)) ? row : highestEquipletScoreIndex;
			}
			
			if(highestEquipletScoreIndex < 0){
				System.out.println("No suitable equiplet found for this step! Scheduling has gone wrong.. Reschedule?");
				return;
			}
			
			//Can we assume that all productSteps are ordered? What about parallel steps? Lets get the equiplet.
			Equiplet equiplet = equiplets[highestEquipletScoreIndex]; //this might not work.
			
			//we still need a transportstep?
			currentTimeSlot += grid.getDistanceMatrix().get(1, 1);
			
			//Get first free timeslot
			TimeSlot timeSlot = equiplet.getFirstFreeTimeSlot(currentTimeSlot, productStep.getCapability().getDuration());
			
			//Check the equiplets schedule. Lets check if the schedule fits. 
			//TODO Keep in mind that the deadline is met.
			finalSchedules.put(productStep, new Schedule(timeSlot, equiplet));
			
			//add the time to the currenttimeslot
			currentTimeSlot += productStep.getCapability().getDuration();
		}
		// Message all the equiplets with their correspondig equiplet steps
		for (ProductStep step : finalSchedules.keySet()) {
			Schedule schedule = finalSchedules.get(step);
			schedule.getEquiplet().schedule(step, schedule.getTimeSlot());
		}
	}
	
	private void setSequenceValues(int row, int firstInSequence, int sequenceLength, Matrix matrix){
		int value = sequenceLength -1;
		for(int i = firstInSequence; i <= sequenceLength; i++){
			matrix.set(row, i, (matrix.get(row, i) + value));
		}
	}
	
	public void reschedule(boolean fromStart){
		ArrayList<ProductStep> newProductSteps = new ArrayList<ProductStep>();
		//only cancel future steps. Lets assume that steps that are already completed are still usable.
		for (ProductStep step : finalSchedules.keySet()) {
			Schedule schedule = finalSchedules.get(step);
			if(fromStart || (!fromStart && !step.isFinished())){
				schedule.getEquiplet().removeFromSchedule(step);
				finalSchedules.remove(step);
				newProductSteps.add(step);
			}
		}
		long currentTimeSlot = TimeSlot.getCurrentTimeSlot();
		//so now we have a newProductSteps and finalSchedules. Lets try to schedule again.
		schedule(currentTimeSlot, generateScheduleMatrix(equiplets, (ProductStep[])newProductSteps.toArray(), currentTimeSlot));
	}
}
