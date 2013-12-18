package simulation.mas_entities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import simulation.Updatable;
import simulation.data.Schedule;
import simulation.data.TimeSlot;
import agents.data_classes.Matrix;

public class Product implements Updatable{
	
	private ProductStep[] productSteps;
	private Equiplet[] equiplets;
	private long deadline;
	private LinkedHashMap<ProductStep, Schedule> finalSchedules; //We might want to keep the order of the list.
	
	public Product(ProductStep[] productSteps, long deadline){
		this.productSteps = productSteps;
		this.deadline = deadline;
		finalSchedules = new LinkedHashMap<ProductStep, Schedule>();
		
		schedule(generateScheduleMatrix(equiplets, productSteps));
	}

	/**
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more information see 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user requirements to product' - Leo van Moergestel section 3.2 
	 * @return the generated scheduleMatrix
	 */
	private Matrix generateScheduleMatrix(Equiplet[] equiplets, ProductStep[] productSteps) {		
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

	private void schedule(Matrix scheduleMatrix) {
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
			
			//Get first free timeslot
			TimeSlot timeSlot = equiplet.getFirstFreeTimeSlot(productStep.getDuration());
			
			//Check the equiplets schedule. Lets check if the schedule fits. Keep in mind that the deadline is met.
			if(equiplet.isScheduleLocked() && timeSlot != null) {
				finalSchedules.put(productStep, new Schedule(timeSlot, equiplet));
			}
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
		//so now we have a newProductSteps and finalSchedules. Lets try to schedule again.
		schedule(generateScheduleMatrix(equiplets, (ProductStep[])newProductSteps.toArray()));
	}

	@Override
	public void update(long time) {
		// TODO Auto-generated method stub
		
	}
}
