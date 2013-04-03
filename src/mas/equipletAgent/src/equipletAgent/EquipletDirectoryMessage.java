package equipletAgent;

import jade.core.AID;
import java.util.ArrayList;

public class EquipletDirectoryMessage {
	public AID AID;
	public ArrayList<Long> capabilities;
	public Object db;

	public EquipletDirectoryMessage(AID AID, ArrayList<Long> capabilities, Object db){
		this.AID = AID;
		this.capabilities = capabilities;
		this.db = db;
	}
}
