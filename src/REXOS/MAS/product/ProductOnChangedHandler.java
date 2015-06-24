package MAS.product;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.json.JSONException;
import org.json.JSONObject;

import MAS.equiplet.EquipletOnChangedHandler.OnChangedTypes;
import MAS.util.Position;
import MAS.util.Tick;
import jade.core.AID;
import jade.lang.acl.ACLMessage;

public class ProductOnChangedHandler {
	
	enum changeType {
		ALL,
		ON_PRODUCT_STATE_CHANGED,
		ON_PRODUCTSTEPS_CHANGED,
		ON_POSITION_CHANGED,
		ON_DEADLINE_CHANGED,
		ON_HISTORY_CHANGED,
		ON_PRODUCTIONPATH_CHANGED
	}
	
	private static final String REGISTER = "REGISTER_LISTENER";
	private static final String DEREGISTER = "DEREGISTER_LISTENER";
	
	private Map<changeType, Set<AID>> productListeners;
	private ProductAgent productAgent;
	
	public ProductOnChangedHandler(ProductAgent productAgent) {
		this.productAgent = productAgent;
		productListeners = new HashMap<changeType, Set<AID>>();
	}
	
	public void handleListenerCommand(ACLMessage msg) {
		if(msg != null) {
			JSONObject request;
			try {
				request = new JSONObject(msg.getContent());
				String command = request.getString("command");
				String action = request.getString("action");
				
				boolean isSuccesfullyAdded = false;
				boolean isValidChangedType = false;
				changeType type = null;
				
				for(changeType t : changeType.values()) {
					if(t.toString().equals(command)) {
						type = t;
						isValidChangedType = true;
					}
				}
				
				if(isValidChangedType) {
					if(action.equals(REGISTER)) {
						isSuccesfullyAdded = registerListener(msg.getSender(),type);
					} else if(action.equals(DEREGISTER)) {
						isSuccesfullyAdded = deregisterListener(msg.getSender(),type);
					}
					
					ACLMessage reply = msg.createReply();
					JSONObject replyMessage = new JSONObject();
					try {
						replyMessage.put("Request", new JSONObject(msg.getContent()));
					} catch (JSONException e) {
						e.printStackTrace();
					}
					reply.setContent(replyMessage.toString());
					
					if(isSuccesfullyAdded){					
						reply.setPerformative(ACLMessage.ACCEPT_PROPOSAL);
					}else {
						reply.setPerformative(ACLMessage.REJECT_PROPOSAL);					
					}
					this.productAgent.send(reply);
				} else {
					ACLMessage reply = msg.createReply();
					JSONObject replyMessage = new JSONObject();
					try {
						replyMessage.put("Request", new JSONObject(msg.getContent()));
					} catch (JSONException e) {
						e.printStackTrace();
					}
					reply.setContent(replyMessage.toString());
					reply.setPerformative(ACLMessage.NOT_UNDERSTOOD);	
					this.productAgent.send(reply);
				}
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	public boolean registerListener(AID aid, changeType t) {
		boolean result = false;
		if(t.toString().equals(changeType.ALL)) {
			for(changeType type : changeType.values()) {
				if(!type.equals(changeType.ALL)) {
					result = addProductToMap(aid, type);
				}
			}
		} else {
			result = addProductToMap(aid, t);
		}
		return result;
	}
	
	public boolean deregisterListener(AID aid, changeType t) {
		boolean isRemovedSuccesfully = false;
		if(t.equals(OnChangedTypes.ALL)){
			for(changeType types : changeType.values()){
				if(!types.equals(changeType.ALL)){
					isRemovedSuccesfully = removeProductFromMap(aid, types);	
				}
			}
		} else {		
			isRemovedSuccesfully = removeProductFromMap(aid, t);			
		}
		return isRemovedSuccesfully;	
	}
	
	public void onProductStepChanged(ACLMessage msg, String stepType){
		switch(stepType){
		case "finished":
			//TODO Implement what SCADA wants to know when this happens
			break;
		case "update":
			//TODO Implement what SCADA wants to know when this happens
			break;
		case "delayed":
			//TODO Implement what SCADA wants to know when this happens
			break;
		}
	}
	
	public void onProductStateChanged(ProductState state){
		//TODO Implement what SCADA wants to know when this happens
	}
	
	public void onProductPositionChanged(Position pos){
		//TODO Implement what SCADA wants to know when this happens
	}
	
	public void onProductDeadlineChanged(Tick deadline){
		//TODO Implement what SCADA wants to know when this happens
	}
	
	public void onProductHistoryChanged(ArrayList<ProductionStep> history){
		//TODO Implement what SCADA wants to know when this happens
	}
	
	private boolean addProductToMap(AID sender, changeType type) {
		boolean result = false;
		if(!productListeners.containsKey(type)){
			productListeners.put(type, new HashSet<AID>());
			productListeners.get(type).add(sender);
			result = true;
		} else {
			if(!productListeners.get(type).contains(sender)){
				productListeners.get(type).add(sender);
				result = true;
			}
		}
		return result;
	}
	
	private boolean removeProductFromMap(AID sender, changeType type) {
		boolean result = false;
		if(productListeners.containsKey(type)){
			if(productListeners.get(type).contains(sender)){
				productListeners.get(type).remove(sender);
				result = true;
			}
		}
		return result;
	}
}
