package scheduling;

//import blackboard;

public class Scheduler {
	
	public Scheduler(Object[] equipletList, Object productionStep){
		//blackboard.connect(scheduleBlackboard);
		
		//TODO tijdsloten berekenen welke vrij zijn	en eerste vrije slot kiezen	
		int startTimeSlot = 0;
		
		schedule(equipletList[0], startTimeSlot);
		
	}
	
	private void schedule(Object equiplet, int startTimeslot){
		//blackboard.place(equipletID, duration);
	}
}
