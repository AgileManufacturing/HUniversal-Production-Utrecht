package simulation.mas_entities;

public class EquipletError {

	public int type;  // 0 == hardware error , 1 == software error ( schedule lost ) 
	
	public long duration;
	
	public long timeOfInterrupt;
	
	public EquipletError(int type, long duration, long timeOfInterrupt){
		this.type = type;
		this.duration = duration;
		this.timeOfInterrupt = timeOfInterrupt;
	}
}
