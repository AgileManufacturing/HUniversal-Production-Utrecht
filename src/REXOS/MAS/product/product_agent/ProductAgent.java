package MAS.product.product_agent;

import generic.ProductStep;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;

import MAS.agents.data_classes.MessageType;
import MAS.agents.data_classes.Proposal;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONTokener;
import org.json.JSONObject;

import util.configuration.ServerConfigurations;

/**
 * ProductAgent that communicates with equipletagents to plan its product steps.
 **/
public class ProductAgent extends Agent {
	private static final String SUCCESSFULL_PLANNED_PRODUCTSTEP = "true";
	private static final String PRODUCT_STEPS = "productSteps";

	private static final long serialVersionUID = 1L;

	private JSONArray productStepList;
	private int currentPlannedProductStep = 0;
	private boolean supplied = false;

	protected void setup() {
		System.out.println("Hello. My name is " + this.getLocalName());
		Object[] arguments = this.getArguments();
		if (arguments.length <= 0) {
			System.out
					.println("No arguments received! Expected product steps in json format");
		} else {
			try {
				JSONObject productSteps = new JSONObject(new JSONTokener(
						arguments[0].toString()));
				if (productSteps != null) {
					productStepList = productSteps.getJSONArray(PRODUCT_STEPS);
					System.out.println("Productagent received product steps: "
							+ productSteps.toString());
					planCurrentProductStep();
				}
			} catch (JSONException ex) {
				System.out.println("Invalid JSON format! " + ex);
			}
		}
		addBehaviour(new CyclicBehaviour() {
			private static final long serialVersionUID = 1L;
			private ArrayList<Proposal> proposals = new ArrayList<Proposal>();

			public void action() {
				try {
					ACLMessage msg = receive();
					if (msg != null) {
						System.out.println(msg.getSender().getName()
								+ " Send: " + msg.getContent());
						if(msg.getContent().equals("Ping")){
							sendPingResponse(msg.getSender());
						}
						else {
							if (msg.getPerformative() == MessageType.AVAILABLE_TO_PLAN) {
								JSONObject proposal = new JSONObject(
										new JSONTokener(msg.getContent()));
								proposals.add(new Proposal(proposal, msg
										.getSender()));
								if (proposals.size() == 1) {
									for (int i = 0; i < productStepList.length(); i++) {
										int productStepId = productStepList
												.getJSONObject(i).getInt("id");
										if (productStepId == (proposal
												.getInt("productStepId"))) {
											proposal.put("productStep",
													productStepList.get(i));
											break;
										}
									}
									System.out.println("Sending plan message: "
											+ proposal.toString());
	
									sendMessage(MessageType.PLAN_PRODUCT_STEP,
											getAID(), proposals.get(0)
													.getEquipletAgent(),
											proposal.toString(), "meta");
								}
							} else if (msg.getPerformative() == MessageType.CONFIRM_PLANNED) {
								// If the product step was successfully planned, go
								// to next product step.
								if (msg.getContent().equals(
										SUCCESSFULL_PLANNED_PRODUCTSTEP)) {
									currentPlannedProductStep++;
								}
								proposals.clear();
								planCurrentProductStep();
							} else if (msg.getPerformative() == MessageType.SUPPLIER_REQUEST_REPLY) {
								supplied = true;
								JSONObject productStep = new JSONObject(
										new JSONTokener(msg.getContent()));
								System.out.println(productStep);
								JSONArray newList = new JSONArray();
								for (int i = 0; i < productStepList.length(); i++) {
									if (i != currentPlannedProductStep) {
										newList.put(productStepList.get(i));
									} else {
										newList.put(productStep);
									}
								}
								productStepList = newList;
								planCurrentProductStep();
							} else {
								System.out
										.println("Received message is not any of capabile Performative MessageType! "
												+ "Could not process incomming message: "
												+ msg.getContent());
							}
						}
					}
					block();
				} catch (JSONException ex) {
					ex.printStackTrace();
				}
			}
		});
	}

	private void planCurrentProductStep() throws JSONException {
		if (productStepList.length() > currentPlannedProductStep) {
			System.out.println(productStepList.length());
			ProductStep productStep = (new ProductStep(
					productStepList.getJSONObject(currentPlannedProductStep)));
			if (productStep.getService().getName().equals("place") && !supplied) {
				String message = productStepList.getJSONObject(
						currentPlannedProductStep).toString();
				AID supplierAgent = new AID();
				// HARDCODED
				supplierAgent.addAddresses(ServerConfigurations.GS_ADDRESS);
				supplierAgent.setName("SupplyAgent@145.89.166.82:1099/JADE");
				System.out.println("PA SEND=" + message);
				sendMessage(MessageType.SUPPLIER_REQUEST, getAID(),
						supplierAgent, message, "meta");
			} else {
				DFAgentDescription dfd = new DFAgentDescription();
				ServiceDescription sd = new ServiceDescription();
				sd.setName(productStep.getService().getName());
				sd.setType(productStep.getService().getName());
				dfd.addServices(sd);
				try {
					DFAgentDescription[] equipletAgents;
					equipletAgents = DFService.search(this, dfd);
					JSONObject message = productStepList
							.getJSONObject(currentPlannedProductStep);
					message.put("startTime", currentPlannedProductStep);
					for (int j = 0; j < equipletAgents.length; j++) {
						AID aid = equipletAgents[j].getName();
						sendMessage(MessageType.CAN_EXECUTE_PRODUCT_STEP,
								getAID(), aid, message.toString(), "meta");
					}
					if (equipletAgents.length == 0) {
						System.out
								.println("No equiplets found to execute this service: "
										+ productStep.getService().getName());
					}
				} catch (FIPAException e) {
					System.out.println("DF Search Error");
					e.printStackTrace();
				}
				supplied = false;
			}
		} else {
			System.out.println("No more product steps to plan");
			System.out.println("Product Agent AWAYYYYYYY!!!!");
			this.doDelete();

		}
	}

	private void sendMessage(int messageType, AID sender, AID receiver,
			String content, String language) {
		ACLMessage message = new ACLMessage(messageType);
		message.setSender(sender);
		message.addReceiver(receiver);
		message.setLanguage(language);
		message.setContent(content);
		send(message);
	}
	/*
	 * Reply to a ping request
	 */
	private void sendPingResponse(AID r) {
		ACLMessage aclMessage = new ACLMessage(ACLMessage.CONFIRM);
		aclMessage.addReceiver(r);
		aclMessage.setContent("Pong");
		send(aclMessage);
	}
}
