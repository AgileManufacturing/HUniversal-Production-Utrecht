package simulation.mas_entities;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import simulation.Simulation;
import simulation.Updatable;
import simulation.data.GridProperties;
import simulation.data.ProductStep;
import simulation.data.ProductStep.StepState;
import simulation.data.Schedule;
import simulation.data.TimeSlot;
import agents.data_classes.Matrix;

public class Product implements Updatable{
	
	private ProductStep[] productSteps;
	private Equiplet[] equiplets;
	private long deadline;
	private LinkedHashMap<ProductStep, Schedule> finalSchedules; //We might want to keep the order of the list.
	private Simulation simulation;
	private Grid grid;
	
	public Product(Simulation simulation, Grid grid, ProductStep[] productSteps, long deadline){
		this.productSteps = productSteps;
		this.deadline = deadline;
		this.simulation = simulation;
		this.grid = grid;
		this.equiplets = grid.getEquiplets();
		finalSchedules = new LinkedHashMap<ProductStep, Schedule>();
		
		long currentTimeSlot = TimeSlot.getCurrentTimeSlot(simulation, grid.getGridProperties());
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
			System.out.println("row " + row);
			sequenceLength = 0; firstInSequence = -1; // always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
			
			long scheduleTimeSlot = currentTimeSlot; // scheduletimeslot has to be the same for each equiplet.
			
			for (int column = 0; column < productSteps.length; column++) { // column ( product steps
				System.out.println("col " + column);
				
				double canPerformStepValue = equiplets[row].canPerformStep(productSteps[column].getCapability()) ? 1.0 : 0.0;
				System.out.println("canPerformStepValue " + canPerformStepValue);
				
				if(canPerformStepValue == 1.0) {   //increase sequence counter.
					scheduleMatrix.set(row, column, canPerformStepValue);
					if(firstInSequence < 0){	  //set the first item in the sequence.
						firstInSequence = column;
					}
					System.out.println("firstInSequence " + firstInSequence);
					sequenceLength++;
					System.out.println("sequenceLength " + sequenceLength);
					if(column == productSteps.length - 1){ // end of row
						System.out.println("at end of row" );
						setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
					}
				} else if(canPerformStepValue == 0.0 && sequenceLength > 0) { // end of sequence
					System.out.println("at end of squence" );
					setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
					sequenceLength = 0;
					firstInSequence = -1;
				}
				scheduleMatrix.show();
				
				//value might have changed since we added sequence multiplier 
				double loadValue = equiplets[row].getLoad(equiplets[row].getFirstFreeTimeSlot(scheduleTimeSlot, productSteps[column].getDuration()));
				System.out.println("loadValue " + loadValue);
				
				//Multiply with load value ( e.g. the load of the equiplet )
				scheduleMatrix.set(row, column, (scheduleMatrix.get(row, column) * (1 - loadValue)));
				
				//we still need a transportstep?
				scheduleTimeSlot += grid.GetMeanDistance(); // this isnt the way it should be done. but it should suffice for now.
				
				//add the time to the scheduleTimeSlot
				scheduleTimeSlot += productSteps[column].getCapability().getDuration();
			}
		}
		scheduleMatrix.show();
		return scheduleMatrix;
	}

	private void schedule(long currentTimeSlot, Matrix scheduleMatrix) {
		// Read the matrix. Write function to iterate each seperate row ( productsteps ) and pick each equiplet 
		Equiplet previousEquiplet, currentEquiplet = null;
		for (int column = 0; column < scheduleMatrix.getNumberOfColumns(); column++) { //Productsteps 
			
			int highestEquipletScoreIndex = -1;
			ProductStep productStep = productSteps[column];
			
			for (int row = 0; row < scheduleMatrix.getNumberOfRows(); row++) { //AID'S
				if(highestEquipletScoreIndex == -1) {
					if(scheduleMatrix.get(row, column) > 0) {
						highestEquipletScoreIndex = row;
					}
				} else {
					highestEquipletScoreIndex = (scheduleMatrix.get(row, column) > scheduleMatrix.get(highestEquipletScoreIndex, column)) ? row : highestEquipletScoreIndex;
				}
			}
			
			if(highestEquipletScoreIndex < 0){
				System.out.println("No suitable equiplet found for this step! Scheduling has gone wrong.. Reschedule?");
				return;
			}
			
			if(currentEquiplet == null){ // first iteration
				currentEquiplet = equiplets[0];
			}
			
			//Can we assume that all productSteps are ordered? What about parallel steps? Lets get the equiplet.
			previousEquiplet = currentEquiplet;
			currentEquiplet = equiplets[highestEquipletScoreIndex]; //this might not work.
			
			//Get first free timeslot
			TimeSlot timeSlot = currentEquiplet.getFirstFreeTimeSlot(currentTimeSlot, productStep.getCapability().getDuration());
			
			//Check the equiplets schedule. Lets check if the schedule fits. 
			//TODO Keep in mind that the deadline is met.
			finalSchedules.put(productStep, new Schedule(timeSlot, currentEquiplet));
			
			//add the time to the currenttimeslot
			currentTimeSlot += productStep.getCapability().getDuration();
			
			//transportdistance
			currentTimeSlot += grid.getDistanceBetweenEquiplets(previousEquiplet, currentEquiplet);
		}
		// Message all the equiplets with their correspondig equiplet steps
		for (ProductStep step : finalSchedules.keySet()) {
			Schedule schedule = finalSchedules.get(step);
			schedule.getEquiplet().schedule(step, schedule.getTimeSlot());
		}
	}
	
	private void setSequenceValues(int row, int firstInSequence, int sequenceLength, Matrix matrix){
		int value = sequenceLength -1;
		for(int i = firstInSequence; i < firstInSequence + sequenceLength; i++){
			System.out.println("setting " + row + " " + i + " to " + (matrix.get(row, i) + value));
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
		long currentTimeSlot = TimeSlot.getCurrentTimeSlot(simulation, grid.getGridProperties());
		//so now we have a newProductSteps and finalSchedules. Lets try to schedule again.
		schedule(currentTimeSlot, generateScheduleMatrix(equiplets, (ProductStep[])newProductSteps.toArray(), currentTimeSlot));
	}
	
	public void updateStep(ProductStep productStep, StepState stepState){
		//wat moet ik nu doen met deze update?
		switch(stepState){
		case Working:
		case Finished:
		case Scheduled:
		case Evaluating:
			productStep.setState(stepState);
		break;
		case ScheduleError:
		case ProductError:
		}
	}

	@Override
	public void update(long time) {
		// TODO Auto-generated method stub
		
	}
}
