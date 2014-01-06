package simulation.mas_entities;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import simulation.Simulation;
import simulation.Updatable;
import simulation.data.Capability;
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
	private Batch batch;
	private boolean needNewSchedule;
	
	public Product(Simulation simulation, Grid grid, Capability[] capabilities, long deadline){
		this(simulation, grid, capabilities, deadline, null);
	}
	public Product(Simulation simulation, Grid grid, Capability[] capabilities, long deadline, Batch batch){
		this.productSteps = generateProductSteps(capabilities);
		this.deadline = deadline;
		this.simulation = simulation;
		this.grid = grid;
		this.batch = batch;
		if(batch == null) {
			this.equiplets = grid.getEquipletsWithoutReservation();
		} else {
			this.equiplets = grid.getEquipletsForReservation(batch.getBatchGroup());
		}
		finalSchedules = new LinkedHashMap<ProductStep, Schedule>();
		
		long currentTimeSlot = TimeSlot.getCurrentTimeSlot(simulation, grid.getGridProperties());
		//We need to pass the current timeslot, to prevent synchronisation issues.
		schedule(currentTimeSlot, generateScheduleMatrix(equiplets, productSteps, currentTimeSlot));
	}
	
	private ProductStep[] generateProductSteps(Capability[] capabilities){
		ProductStep[] productSteps = new ProductStep[capabilities.length];
		for (int i = 0; i < capabilities.length; i++) {
			productSteps[i] = new ProductStep(this, capabilities[i]); 
		}
		return productSteps;
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
			//System.out.println("row " + row);
			sequenceLength = 0; firstInSequence = -1; // always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
			
			long scheduleTimeSlot = currentTimeSlot; // scheduletimeslot has to be the same for each equiplet.
			
			for (int column = 0; column < productSteps.length; column++) { // column ( product steps
				//System.out.println("col " + column);
				
				double canPerformStepValue = equiplets[row].canPerformStep(productSteps[column].getCapability()) ? 1.0 : 0.0;
				//System.out.println("canPerformStepValue " + canPerformStepValue);
				
				if(canPerformStepValue == 1.0) {   //increase sequence counter.
					scheduleMatrix.set(row, column, canPerformStepValue);
					if(firstInSequence < 0){	  //set the first item in the sequence.
						firstInSequence = column;
					}
					sequenceLength++;
					if(column == productSteps.length - 1){ // end of row
						setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
					}
				} else if(canPerformStepValue == 0.0 && sequenceLength > 0) { // end of sequence
					setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
					sequenceLength = 0;
					firstInSequence = -1;
				}
				TimeSlot loadSlot = equiplets[row].getFirstFreeTimeSlot(scheduleTimeSlot, productSteps[column].getDuration());
				
				//value might have changed since we added sequence multiplier 
				double loadValue = equiplets[row].getLoad(loadSlot);
				
				//Multiply with load value ( e.g. the load of the equiplet )
				scheduleMatrix.set(row, column, (scheduleMatrix.get(row, column) * (1 - loadValue)));

				//add the time to the scheduleTimeSlot
				scheduleTimeSlot += productSteps[column].getCapability().getDuration();
				
				//we still need a transportstep?
				scheduleTimeSlot += grid.GetMeanDistance(); 
			}
		}
		System.out.println("Product {" + this + "} schedule: ");
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
			
			//transportdistance
			currentTimeSlot += grid.getDistanceBetweenEquiplets(previousEquiplet, currentEquiplet);
			
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
			
			if(schedule.getEquiplet().schedule(step, schedule.getTimeSlot())){
				step.setState(StepState.Scheduled);
			} else {
				needNewSchedule = true;
			}
			
		}
	}
	
	private void setSequenceValues(int row, int firstInSequence, int sequenceLength, Matrix matrix){
		int value = sequenceLength -1;
		for(int i = firstInSequence; i < firstInSequence + sequenceLength; i++){
			matrix.set(row, i, (matrix.get(row, i) + value));
		}
	}
	
	private void reschedule(boolean fromStart){
		ArrayList<ProductStep> newProductSteps = new ArrayList<ProductStep>();
		//only cancel future steps. Lets assume that steps that are already completed are still usable.
		
		ProductStep[] steps = finalSchedules.keySet().toArray(new ProductStep[finalSchedules.size()]);
		for(int i = 0; i < finalSchedules.size(); i++) {
			ProductStep step = steps[i];
			Schedule schedule = finalSchedules.get(step);
			if(fromStart || (!fromStart && !step.isFinished())){
				schedule.getEquiplet().removeFromSchedule(step);
				finalSchedules.remove(step);
				newProductSteps.add(step);
			}
		}
		long currentTimeSlot = TimeSlot.getCurrentTimeSlot(simulation, grid.getGridProperties());
		//so now we have a newProductSteps and finalSchedules. Lets try to schedule again.
		schedule(currentTimeSlot, generateScheduleMatrix(equiplets, 
				newProductSteps.toArray(new ProductStep[newProductSteps.size()]), currentTimeSlot));
	}
	
	public void handleEquipletError(StepState stepState){
		if(stepState == StepState.ScheduleError){
			reschedule(false);
		}
		if(stepState == StepState.ProductError){
			reschedule(true);
		}
	}

	@Override
	public void update(long time) {
		if(needNewSchedule == true) {
			needNewSchedule = false;
			reschedule(true);
		}
	}
	public long getDeadline() {
		return deadline;
	}
	public LinkedHashMap<ProductStep, Schedule> getFinalSchedules() {
		return finalSchedules;
	}
	public String toString() {
		if(batch != null) {
			return "id:" + this.hashCode() + " batch:" + batch.getBatchGroup();
		} else {
			return "id:" + this.hashCode() + " batch:-";
		}
	}
}
