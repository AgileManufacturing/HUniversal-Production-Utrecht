package MAS.product;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import jade.lang.acl.ACLMessage;

public class ProductGetDataHandler {
	
	ProductAgent productAgent;
	
	public ProductGetDataHandler(ProductAgent productAgent) {
		this.productAgent = productAgent;
	}
	
	public void handleGetDataRequest(ACLMessage msg) {
		if(msg != null) {
			JSONObject reply = new JSONObject();
			try {
				JSONObject messageContent = new JSONObject(msg.getContent());
				String command = messageContent.getString("command");
				reply.put("command", command);
				JSONObject agent = new JSONObject();
				agent.put("id", this.productAgent.getAID().getLocalName());
				
				switch(command) {
					case ProductCommand.COMMAND_GET_BASIC_INFO: 
						agent.put("type", "ProductAgent");
						agent.put("state", this.productAgent.getProductState().name());
						break;
					case ProductCommand.COMMAND_GET_DETAILED_INFO:
						agent = this.getDetailedInfo();
					case ProductCommand.COMMAND_GET_TYPE:
						agent.put("type", "ProductAgent");
						break;
					case ProductCommand.COMMAND_GET_CURRENT_STATE:
						agent.put("state", this.productAgent.getProductState().name());
						break;
					case ProductCommand.COMMAND_GET_DEADLINE:
						agent.put("deadline", this.productAgent.getDeadline().toString());
						break;
					case ProductCommand.COMMAND_GET_POSITION:
						agent.put("position", this.productAgent.getPosition().toString());
						break;
					case ProductCommand.COMMAND_GET_CURRENT_STEP:
						agent.put("current-step", this.productAgent.getCurrentStep().toString());
						break;
					case ProductCommand.COMMAND_GET_CREATED:
						agent.put("created", this.productAgent.getCreated().toString());
						break;
					case ProductCommand.COMMAND_GET_PRODUCTIONSTEPS:
						JSONArray steps = new JSONArray();
						for(int i = 0; i < this.productAgent.getProductSteps().size(); i++) {
							steps.put(this.productAgent.getProductSteps().get(i));
						}
						agent.put("steps", steps);
						break;
					case ProductCommand.COMMAND_GET_PRODUCTIONPATH:
						JSONArray path = new JSONArray();
						for(int i = 0; i < this.productAgent.getProductionPath().size(); i++) {
							path.put(this.productAgent.getProductionPath().get(i));
						}
						agent.put("path", path);
						break;
					case ProductCommand.COMMAND_GET_HISTORY:
						JSONArray history = new JSONArray();
						for(int i = 0; i < this.productAgent.getHistory().size(); i++) {
							history.put(this.productAgent.getHistory().get(i));
						}
						agent.put("history", history);
						break;
					default:
						reply = null;
						System.out.println("No suitable command found!");
						break;
				}
				if(reply != null) {
					reply.put("agent", agent);
				}
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			ACLMessage response = msg.createReply();
			if(reply != null) {
				response.setContent(reply.toString());
				response.setPerformative(ACLMessage.INFORM);
			} else {
				response.setPerformative(ACLMessage.FAILURE);
			}
			this.productAgent.send(response);
		}
	}
	
	private JSONObject getDetailedInfo() {
		JSONObject agent = new JSONObject();
		try {
			agent.put("id", this.productAgent.getAID().getLocalName());
			agent.put("type", "ProductAgent");
			agent.put("state", this.productAgent.getProductState().name());
			agent.put("deadline", this.productAgent.getDeadline().toString());
			agent.put("position", this.productAgent.getPosition().toString());
			agent.put("current-step", this.productAgent.getCurrentStep().toString());
			agent.put("created", this.productAgent.getCreated().toString());
			JSONArray steps = new JSONArray();
			for(int i = 0; i < this.productAgent.getProductSteps().size(); i++) {
				steps.put(this.productAgent.getProductSteps().get(i));
			}
			agent.put("steps", steps);
			JSONArray path = new JSONArray();
			for(int i = 0; i < this.productAgent.getProductionPath().size(); i++) {
				path.put(this.productAgent.getProductionPath().get(i));
			}
			agent.put("path", path);
			JSONArray history = new JSONArray();
			for(int i = 0; i < this.productAgent.getHistory().size(); i++) {
				history.put(this.productAgent.getHistory().get(i));
			}
			agent.put("history", history);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return agent;
	}
}
