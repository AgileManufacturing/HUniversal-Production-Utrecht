package libraries.schedule;

import java.util.UUID;

import libraries.blackboard_client.BlackboardClient;
import libraries.schedule.data_classes.Schedule;
import libraries.schedule.data_classes.ScheduleAccessException;
import libraries.schedule.data_classes.ScheduleLock;

public class EquipletSchedule extends Schedule {
	
	
	
	private BlackboardClient planningBlackboard;
	
	private BlackboardClient FreeTimeSlotBlackboard;
	
	private BlackboardClient RealtimeBlackboard;
	
	public EquipletSchedule(String scheduleHostName, int schedulePort, boolean clearSchedule){
		super(scheduleHostName, schedulePort);
		Setup();
		
		if (clearSchedule){
			ClearSchedule();
		}
	}
	
	private void Setup(){
		
	}
	
	private void ClearSchedule(){
		
	}

	@Override
	public void GetFreeTimeSlots(UUID lockKey) throws ScheduleAccessException {
		super.GetFreeTimeSlots(lockKey);
		// TODO Auto-generated method stub
	}

	@Override
	public void ScheduleOn(UUID lockKey) throws ScheduleAccessException {
		super.ScheduleOn(lockKey);
		// TODO Auto-generated method stub
		
	}

}
