package simulation.mas_entities;

import simulation.data.Capability;

public class ProductStep {
	private Capability capability;
	private long duration;
	
	public ProductStep(Capability capability){
		this.capability = capability;
	}

	public long getDuration() {
		return duration;
	}

	public void setDuration(long duration) {
		this.duration = duration;
	}
	
	public Capability getCapability(){
		return capability;
	}
	
}
