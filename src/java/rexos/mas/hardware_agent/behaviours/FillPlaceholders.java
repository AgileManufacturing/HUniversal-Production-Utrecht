package rexos.mas.hardware_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.util.List;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.hardware_agent.EquipletStepMessage;
import rexos.mas.hardware_agent.HardwareAgent;
import rexos.mas.hardware_agent.Module;
import rexos.mas.hardware_agent.ModuleFactory;
import rexos.mas.service_agent.ServiceStepMessage;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class FillPlaceholders extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("FillPlaceholders");
	private HardwareAgent hardwareAgent;
	private ModuleFactory moduleFactory;

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a
	 *            the a
	 */
	public FillPlaceholders(Agent a, ModuleFactory moduleFactory) {
		super(a, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
		this.moduleFactory = moduleFactory;
	}

	@Override
	public void handle(ACLMessage message) {
		try{
			ObjectId serviceStepId = (ObjectId) message.getContentObject();
			Logger.log("%s received message from %s %n", myAgent.getLocalName(), message.getSender().getLocalName(),
					message.getOntology());
			FillStepPlaceholders(serviceStepId);
		} catch(UnreadableException e) {
			Logger.log(e);
			myAgent.doDelete();
		}
	}

	public void FillStepPlaceholders(ObjectId serviceStepId){
		try {
			ServiceStepMessage serviceStep = new ServiceStepMessage(
					(BasicDBObject) hardwareAgent.getServiceStepsBBClient().findDocumentById(serviceStepId));
			BlackboardClient equipletStepBBClient = hardwareAgent.getEquipletStepsBBClient();
			BasicDBObject query = new BasicDBObject("serviceStepID", serviceStep.getId());

			List<DBObject> steps = equipletStepBBClient.findDocuments(query);
			EquipletStepMessage[] equipletSteps = new EquipletStepMessage[steps.size()];
			for(int i = 0; i < steps.size(); i++) {
				equipletSteps[i] = new EquipletStepMessage((BasicDBObject) steps.get(i));
			}

			int leadingModule = hardwareAgent.getLeadingModule(serviceStep.getServiceId());
			Module module = moduleFactory.getModuleById(leadingModule);
			module.setConfiguration(hardwareAgent.getConfiguration());

			equipletSteps = module.fillPlaceHolders(equipletSteps, serviceStep.getParameters());
			for(EquipletStepMessage step : equipletSteps) {
				equipletStepBBClient.updateDocuments(new BasicDBObject("_id", step.getId()), step.toBasicDBObject());
			}
			if(serviceStep.getNextStep() != null){
				FillStepPlaceholders(serviceStep.getNextStep());
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
			myAgent.doDelete();
		}
	}
}
