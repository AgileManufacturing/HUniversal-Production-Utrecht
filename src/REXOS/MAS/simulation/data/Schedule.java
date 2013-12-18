package simulation.data;

import simulation.mas_entities.Equiplet;
import simulation.mas_entities.ProductStep;

public class Schedule {
	private ProductStep productStep;
	private TimeSlot timeSlot;
	private Equiplet equiplet;
	
	public Schedule(ProductStep productStep, TimeSlot timeSlot,
			Equiplet equiplet) {
		this.productStep = productStep;
		this.timeSlot = timeSlot;
		this.equiplet = equiplet;
	}
	
	public ProductStep getProductStep() {
		return productStep;
	}
	public void setProductStep(ProductStep productStep) {
		this.productStep = productStep;
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
