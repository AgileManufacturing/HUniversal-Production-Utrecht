package libraries.schedule.data_classes;

import java.util.ArrayList;

public class FreeTimeData {

	
	private ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
	
	private long infiniteFreeTimeSlot;
	
	public FreeTimeData(ArrayList<FreeTimeSlot> freeTimeSlots, long infiniteFreeTimeSlot){
		this.freeTimeSlots = freeTimeSlots;
		this.infiniteFreeTimeSlot = infiniteFreeTimeSlot;
	}
	
	public ArrayList<FreeTimeSlot> getFreeTimeSlots(){
		return freeTimeSlots;
	}
	
	public long getinfiniteFreeTimeSlot(){
		return infiniteFreeTimeSlot;
	}
}
