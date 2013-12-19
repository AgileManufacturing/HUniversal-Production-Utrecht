package simulation.data;


public class ProductStepSchedule {
	
	private ProductStep productStep;

	private TimeSlot timeSlot;
	
	public ProductStepSchedule(ProductStep productStep, TimeSlot timeSlot){
		this.productStep = productStep;
		this.timeSlot = timeSlot;
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
	
	public long getStartTimeSlot(){
		return timeSlot.getStartTimeSlot();
	}
	
	public Long getDuration(){
		return timeSlot.getDuration();
	}
	
	
}
