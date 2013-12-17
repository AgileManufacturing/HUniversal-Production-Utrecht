package simulation.mas_entities;

public class EquipletInterrupt {

	public int type;  // 0 == hardware error , 1 == software error ( schedule lost ) 
	
	public long duration;
	
	public long timeOfInterrupt;
	
	public EquipletInterrupt(int type, long duration, long timeOfInterrupt){
		this.type = type;
		this.duration = duration;
		this.timeOfInterrupt = timeOfInterrupt;
	}
}
