package scheduling;

//import blackboard;

public class Scheduler {
	
	public Scheduler(Object[] equipletList, Object productionStep){
		blackboard.connect(scheduleBlackboard);
		int neededTimeSlots = (ProductionStep) productionStep.getTimeSlots();
		
		//TODO tijdSloten berekenen welke vrij zijn		
		
		(ProductionStep)productionStep.schedule(equipletList[0], neededTimeSlots);
		
	}
}
