package simulation.data;

import simulation.mas_entities.Equiplet;

public class Schedule {
	private ProductStep productStep;
	private TimeSlot timeSlot;
	private Equiplet equiplet;
	
	public Schedule(TimeSlot timeSlot,
			Equiplet equiplet) {
		this.timeSlot = timeSlot;
		this.equiplet = equiplet;
	}
	
	public TimeSlot getTimeSlot() {
		return timeSlot;
	}
	public void setTimeSlot(TimeSlot timeSlot) {
		this.timeSlot = timeSlot;
	}
	public Equiplet getEquiplet() {
		return equiplet;
	}
	public void setEquiplet(Equiplet equiplet) {
		this.equiplet = equiplet;
	}
	
	
}
