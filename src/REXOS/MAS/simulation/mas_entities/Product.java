package simulation.mas_entities;

import java.util.ArrayList;

import libraries.schedule.data_classes.FreeTimeSlot;
import libraries.schedule.data_classes.ProductStepSchedule;
import agents.data_classes.Matrix;

public class Product {
	
	private ProductStep[] productSteps;
	private Equiplet[] equiplets;
	private long deadline;
	
	public Product(ProductStep[] productSteps, long deadline){
		this.productSteps = productSteps;
		this.deadline = deadline;
	}
	
	//Get all the feasable equiplets from the 'index' and maybe even start planning
	private void plan(){
		//iterate all steps, and find feasible equiplets.
		//save them, and use them for the next step.
	}

	/**
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more information see 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user requirements to product' - Leo van Moergestel section 3.2 
	 * @return the generated scheduleMatrix
	 */
	private Matrix generateScheduleMatrix() {		
		//construct a new matrices to perform a neat-o selection
		Matrix scheduleMatrix = new Matrix(equiplets.length, productSteps.length);
		
		int sequenceLength, firstInSequence;
		
			//Iterate through them steps to fill the matrix
			for (int row = 0; row < equiplets.length; row++) { // row ( equiplets )
				sequenceLength = 0; firstInSequence = -1; // always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
				
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
					double loadValue = equiplets[row].getLoad();
					
					//TODO: perform supermagic calculation of currentValue * load here
					scheduleMatrix.set(row, column, (scheduleMatrix.get(row, column) * loadValue));
				}
		}
		return scheduleMatrix;
	}

	@SuppressWarnings("unchecked")
	private boolean schedule(Matrix scheduleMatrix) {
		
		// Read the matrix. Write function to iterate each seperate row ( productsteps ) and pick each equiplet 
		for (int column = 0; column < scheduleMatrix.getNumberOfColumns(); column++) { //Productsteps 
			int highestEquipletScoreIndex = -1;
			ProductStep productStep = productSteps[column];
			
			for (int row = 0; row < scheduleMatrix.getNumberOfRows(); row++) { //AID'S
				highestEquipletScoreIndex = (scheduleMatrix.get(row, column) > scheduleMatrix.get(highestEquipletScoreIndex, column)) ? row : highestEquipletScoreIndex;
			}
			
			if(highestEquipletScoreIndex < 0){
				System.out.println("No suitable equiplet found for this step! Scheduling has gone wrong.. Reschedule?");
				return false;
			}
			
			//Can we assume that all productSteps are ordered? What about parallel steps? Lets get the equiplet.
			Equiplet equiplet = equiplets[highestEquipletScoreIndex]; //?lol? this might not work.
			
			//Get first free timeslot
			long ts = equiplet.getFirstFreeTimeSlot();
			
			//Check the equiplets schedule. Lets check if the schedule fits. Keep in mind that the deadline is met.
			if(equiplet.isScheduleLocked() && ts > 0) {
				
				//We should still set the proper deadline. ( none is given for now )
				//finalSchedules.add();
				
				//plan the timeslot
				//scheduleInformation.planTimeSlot(freeTimeSlot);
			}
			
			// If the schedule fits, save the equiplet with corresponding step(s) ( maybe equipletmapper? )
		}
		// Message all the equiplets with their correspondig equiplet steps
		return false;//sendScheduleMessages(finalSchedules);
	}
	
	private void setSequenceValues(int row, int firstInSequence, int sequenceLength, Matrix matrix){
		int value = sequenceLength -1;
		for(int i = firstInSequence; i <= sequenceLength; i++){
			matrix.set(row, i, (matrix.get(row, i) + value));
		}
	}
	
	private void reschedule(boolean fromStart){
		
		if(fromStart){
			//remove myself from all equiplets.
		}
		
	}
}
