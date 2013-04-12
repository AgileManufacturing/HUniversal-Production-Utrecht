package hardwareAgent;

import org.bson.types.ObjectId;

public class EquipletStepMessage {
	public ObjectId serviceStepID;
	public Long constructionData;
	public Long type;
	public TimeData timeData;
	public EquipletStepMessage(ObjectId serviceStepID,
							  Long constructionData, Long type, TimeData timeData){
		
		this.serviceStepID = serviceStepID;
		this.constructionData = constructionData;
		this.timeData = timeData;
		this.type = type;
	}
}
