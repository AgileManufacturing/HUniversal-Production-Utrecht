package simulation.data;

import simulation.Simulation;

public class GridProperties {

	//timedata
	//private long firstTimeSlot = 1387360454938l;
	private long timeSlotLength = 10l;
	
	//equipletdata
	private long equipletLoadWindow = 10000l;
	
	private Simulation simulation;

	public GridProperties(Simulation simulation, long firstTimeSlot, long timeSlotLength, long equipletLoadWindow){
		this.simulation = simulation;
		//this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;
		this.equipletLoadWindow = equipletLoadWindow;
	}
	
	public Simulation getSimulation() {
		return simulation;
	}
	
	public long getFirstTimeSlot() {
		return simulation.getStartSimulationTime();
	}

	public void setFirstTimeSlot(long firstTimeSlot) {
		//this.firstTimeSlot = firstTimeSlot;
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
