package scheduling;

import jade.core.AID;

//import blackboard;

public class Scheduler {
	
	public Scheduler(AID[] equipletList, Object productionStep){
		//blackboard.connect(scheduleBlackboard);
		
		//TODO tijdsloten berekenen welke vrij zijn	en eerste vrije slot kiezen
		//( bepaald ook welke equiplet het wordt indien meerdere equiplets beschikbaar)	
		int startTimeSlot = 0;
		
		schedule(equipletList[0], startTimeSlot);
		
	}
	
	private void schedule(AID equiplet, int startTimeslot){
		blackboard.place(equipletID, startTimeSlot);
	}
}
