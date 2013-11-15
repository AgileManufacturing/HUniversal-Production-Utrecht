package libraries.schedule.data_classes;

import java.io.Serializable;
import java.util.ArrayList;

public class EquipletFreeTimeData implements Serializable {

	/**
	 * @var long serialVersionUID
	 * 		The serialization UID of this class
	 */
	private static final long serialVersionUID = 6144606966764813921L;

	private ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
	
	private long infiniteFreeTimeSlot;
	
	private double load;
	
	public EquipletFreeTimeData(ArrayList<FreeTimeSlot> freeTimeSlots, long infiniteFreeTimeSlot, double load){
		this.freeTimeSlots = freeTimeSlots;
		this.infiniteFreeTimeSlot = infiniteFreeTimeSlot;
	}
	
	public ArrayList<FreeTimeSlot> getFreeTimeSlots(){
		return freeTimeSlots;
	}
	
	public long getinfiniteFreeTimeSlot(){
		return infiniteFreeTimeSlot;
	}
	
	public double getLoad(){
		return load;
	}
}
