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
import rexos.mas.service_agent.ServiceStep;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

/**
 * Class for the receivebehaviour receiving messages with the ontology FillPlaceholders.
 */
public class FillPlaceholders extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 * The serialVersionUID.
	 */
	private static final long serialVersionUID = 1L;
	
	/**
	 * @var MessageTemplate messageTemplate
	 * The messageTemplate to match the messages to.
	 */
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("FillPlaceholders");
	
	/**
	 * @var HardwareAgent hardwareAgent
	 * The hardwareAgent for this behaviour.
	 */
	private HardwareAgent hardwareAgent;
	/**
	 * @var ModuleFactory moduleFactory
	 * The moduleFactory
	 */
	private ModuleFactory moduleFactory;

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a the agent
	 * @param moduleFactory the moduleFactory
	 */
	public FillPlaceholders(Agent a, ModuleFactory moduleFactory) {
		super(a, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
		this.moduleFactory = moduleFactory;
	}

	/**
	 * @see ReceiveBehaviour#handle(ACLMessage)
	 */
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

	/**
	 * Function for filling the placeholders of the equipletSteps for an serviceStepId
	 * @param serviceStepId The serviceStepId to fill the equipletsSteps for.
	 */
	public void FillStepPlaceholders(ObjectId serviceStepId){
		try {
			//Get the serviceStep
			ServiceStep serviceStep = new ServiceStep(
					(BasicDBObject) hardwareAgent.getServiceStepsBBClient().findDocumentById(serviceStepId));
			BlackboardClient equipletStepBBClient = hardwareAgent.getEquipletStepsBBClient();
			BasicDBObject query = new BasicDBObject("serviceStepID", serviceStep.getId());
			//Get the equipletSteps
			List<DBObject> steps = equipletStepBBClient.findDocuments(query);
			EquipletStepMessage[] equipletSteps = new EquipletStepMessage[steps.size()];
			for(int i = 0; i < steps.size(); i++) {
				equipletSteps[i] = new EquipletStepMessage((BasicDBObject) steps.get(i));
			}
			
			//Get the leadingModule
			int leadingModule = hardwareAgent.getLeadingModule(serviceStep.getServiceId());
			Module module = moduleFactory.getModuleById(leadingModule);
			module.setConfiguration(hardwareAgent.getConfiguration());

			//Fill the placeholders
			equipletSteps = module.fillPlaceHolders(equipletSteps, serviceStep.getParameters());
			for(EquipletStepMessage step : equipletSteps) {
				equipletStepBBClient.updateDocuments(new BasicDBObject("_id", step.getId()), step.toBasicDBObject());
			}
			//if the serviceStep has a nextStep fill the placeholders for that one to.
			if(serviceStep.getNextStep() != null){
				FillStepPlaceholders(serviceStep.getNextStep());
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
			myAgent.doDelete();
		}
	}
}
