package simulation.data;

public class GridProperties {

	//timedata
	private long firstTimeSlot = 1387360454938l;
	private long timeSlotLength = 10l;
	
	//equipletdata
	private long equipletLoadWindow = 10000l;

	public GridProperties(long firstTimeSlot, long timeSlotLength, long equipletLoadWindow){
		this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;
		this.equipletLoadWindow = equipletLoadWindow;
	}
	
	public long getFirstTimeSlot() {
		return firstTimeSlot;
	}

	public void setFirstTimeSlot(long firstTimeSlot) {
		this.firstTimeSlot = firstTimeSlot;
	}

	public long getTimeSlotLength() {
		return timeSlotLength;
	}

	public void setTimeSlotLength(long timeSlotLength) {
		this.timeSlotLength = timeSlotLength;
	}

	public long getEquipletLoadWindow() {
		return equipletLoadWindow;
	}

	public void setEquipletLoadWindow(long equipletLoadWindow) {
		this.equipletLoadWindow = equipletLoadWindow;
	}
	
	//gridlayout
	
	
}
