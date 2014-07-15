package grid_server;

import generic.ProductStep;

import java.util.ArrayList;

import agents.data_classes.MessageType;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;

import configuration.ServerConfigurations;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

/**
 * ProductAgent that communicates with equipletagents to plan its product steps.
 **/
public class ProductAgent extends Agent{
	private static final String SUCCESSFULL_PLANNED_PRODUCTSTEP = "true";
	private static final String PRODUCT_STEPS = "productSteps";
	
	private static final long serialVersionUID = 1L;
	
	private JsonArray productStepList;
	private int currentPlannedProductStep = 0;
	private boolean supplied = false;

	protected void setup(){
		Object[] arguments = this.getArguments();
		if (arguments.length <= 0){
			System.out.println("No arguments received! Expected product steps in json format");
		}
		else {
			try {
				JsonElement productSteps = new JsonParser().parse(arguments[0].toString());
				if (productSteps != null){
					productStepList = productSteps.getAsJsonObject().get(PRODUCT_STEPS).getAsJsonArray();
					System.out.println("Productagent received product steps: " + productSteps.toString());
					planCurrentProductStep();
				}
			} catch (JsonSyntaxException ex) {
				System.out.println("Invalid JSON format! " + ex);
			}
		}
		
		
		
		
		
		
		addBehaviour(new CyclicBehaviour() {
			private static final long serialVersionUID = 1L;
			private ArrayList<Proposal> proposals = new ArrayList<Proposal>();

			public void action() {
				ACLMessage msg = receive();
				if (msg!=null) {
					System.out.println(msg.getSender().getName()+" Send: "+msg.getContent() );

					if (msg.getPerformative() == MessageType.AVAILABLE_TO_PLAN){
						JsonObject proposal = new JsonParser().parse(msg.getContent()).getAsJsonObject();
						proposals.add(new Proposal(proposal,msg.getSender()));
						if (proposals.size() == 1){
							for (int i=0;i<productStepList.size();i++){
								String pid = productStepList.get(i).getAsJsonObject().get("id").getAsString();
								if (pid.equals(proposal.get("productStepId").getAsString())){
									proposal.add("productStep",productStepList.get(i));
									i = productStepList.size();
								}
							}
							System.out.println("Sending plan message: " + proposal.toString());
							
				            sendMessage(MessageType.PLAN_PRODUCT_STEP, 
				            			getAID(), 
				            			proposals.get(0).getEquipletAgent(), 
				            			proposal.toString(),
				            			"meta");
						}
					} 
					else if (msg.getPerformative() == MessageType.CONFIRM_PLANNED) {
						//If the product step was successfully planned, go to next product step.
						if (msg.getContent().equals(SUCCESSFULL_PLANNED_PRODUCTSTEP)){
							currentPlannedProductStep++;
						}
						proposals.clear();
						planCurrentProductStep();
					}
					else if(msg.getPerformative() == MessageType.SUPPLIER_REQUEST_REPLY){
						supplied = true;
						JsonElement productStep = new JsonParser().parse(msg.getContent());
						System.out.println(productStep);
						JsonArray newList = new JsonArray();
						for (int i=0;i<productStepList.size();i++){
							if (i != currentPlannedProductStep){
								newList.add(productStepList.get(i));
							}else{
								newList.add(productStep);
							}
						}
						productStepList = newList;
						planCurrentProductStep();
					}
					else {
						System.out.println(	"Received message is not any of capabile Performative MessageType! " + 
											"Could not process incomming message: " + msg.getContent());
					}
				}
				block();    
			}  
		});  
	}
	
	private void planCurrentProductStep(){
		if (productStepList.size() > currentPlannedProductStep){
			System.out.println(productStepList.size());
			ProductStep productStep = (new ProductStep(productStepList.get(currentPlannedProductStep).getAsJsonObject()));
			if(productStep.getService().getName().equals("place") && !supplied){
				String message = productStepList.get(currentPlannedProductStep).getAsJsonObject().toString();
				AID supplierAgent = new AID();
				
				supplierAgent.addAddresses(ServerConfigurations.GS_ADDRESS);
				System.out.println("Supplier adress: "+supplierAgent.getAddressesArray()[0]);
				supplierAgent.setName("SupplyAgent@"+ServerConfigurations.AGENT_ADDRESS);
				System.out.println("PA SEND=" +message);
				sendMessage(MessageType.SUPPLIER_REQUEST, getAID(), supplierAgent, message, "meta");
			}
			else {
				DFAgentDescription dfd = new DFAgentDescription();
				ServiceDescription sd = new ServiceDescription();
				sd.setName(productStep.getService().getName());
				sd.setType(productStep.getService().getName());
				dfd.addServices(sd);
				try {
					DFAgentDescription[] equipletAgents;
					equipletAgents = DFService.search(this, dfd);
					JsonObject message = productStepList.get(currentPlannedProductStep).getAsJsonObject();
					message.addProperty("startTime", currentPlannedProductStep);
					for (int j=0;j<equipletAgents.length;j++){
						AID aid = equipletAgents[j].getName();
						sendMessage(MessageType.CAN_EXECUTE_PRODUCT_STEP, getAID(), aid, message.toString(), "meta");
					}
					if (equipletAgents.length == 0){
						System.out.println("No equiplets found to execute this service: " + productStep.getService().getName());
					}
				} catch (FIPAException e) {
					System.out.println("DF Search Error");
					e.printStackTrace();
				}
				supplied = false;
			}
		}
		else {
			System.out.println("No more product steps to plan");
			System.out.println("Product Agent AWAYYYYYYY!!!!");
			this.doDelete();
			
		}
	}
	
	private void sendMessage(int messageType, AID sender, AID receiver, String content, String language){
		ACLMessage message = new ACLMessage( messageType );
		message.setSender(sender);
		message.addReceiver(receiver);
		message.setLanguage(language);
		message.setContent(content);
		send(message);
	}
}
