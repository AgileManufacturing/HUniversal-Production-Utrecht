package hardwareAgent;
/**
 * Author: Wouter Veen & Thierry Gerritse
 * Class: EquipletStepMessage.java 
 **/
import org.bson.types.ObjectId;

public class EquipletStepMessage {
	public ObjectId serviceStepID;
	public Long instructionData;
	public Long type;
	public TimeData timeData;
	
	/**
	 * @param serviceStepID
	 * @param ionstructionData
	 * @param type
	 * @param timeData
	 * @return
	 */
	public EquipletStepMessage(ObjectId serviceStepID,
							  Long instructionData, Long type, TimeData timeData){
		
		this.serviceStepID = serviceStepID;
		this.instructionData = instructionData;
		this.timeData = timeData;
		this.type = type;
	}
}
