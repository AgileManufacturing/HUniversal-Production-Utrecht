package simulation.collectors;

import simulation.Simulation;

public abstract class DataCollector{
	final long sampleInterval = 5000; // milliseconds
	long lastSample = -1;
	
	Simulation simulation;
	
	DataCollector(Simulation simulation) {
		this.simulation = simulation;
		this.simulation.addDataCollector(this);
		
		
	}
	protected boolean needNewSample(long time) {
		if(time >= lastSample + sampleInterval) {
			if(lastSample == -1) lastSample = time;
			else lastSample += sampleInterval;
			return true;
		}
		return false;
	}
	public abstract void collectData(long time);
}