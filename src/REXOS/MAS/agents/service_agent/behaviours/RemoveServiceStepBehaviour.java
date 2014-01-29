package agents.service_agent.behaviours;

import java.io.IOException;
import java.util.ArrayList;

import org.bson.types.ObjectId;

import agents.equiplet_agent.EquipletAgent;
import agents.service_agent.ServiceAgent;
import agents.service_agent.ServiceStep;
import agents.shared_behaviours.ReceiveBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

public class RemoveServiceStepBehaviour extends ReceiveBehaviour{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 681033710946072342L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate to match the messages.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("RemoveServiceStep");
	
	private ServiceAgent serviceAgent;
	
	public RemoveServiceStepBehaviour(ServiceAgent serviceAgent) {
		super(serviceAgent, MESSAGE_TEMPLATE);
		this.serviceAgent = serviceAgent;
	}

	@Override
	public void handle(ACLMessage message) {
		if (message.getPerformative() == ACLMessage.INFORM){
			try {
				ObjectId productStepId = (ObjectId)message.getContentObject();
				ArrayList<ObjectId> serviceStepIds = serviceAgent.getServiceStepIdsByProductStepId(productStepId);
				
				serviceAgent.removeServiceStepsByProductStepId(productStepId);
				
				ACLMessage removeEquipletStepsAclMessage = new ACLMessage(ACLMessage.INFORM);
				removeEquipletStepsAclMessage.addReceiver(serviceAgent.getHardwareAgentAID());
				removeEquipletStepsAclMessage.setOntology("RemoveEquipletStep");
				removeEquipletStepsAclMessage.setContentObject(serviceStepIds.toArray(new ObjectId[serviceStepIds.size()]));
				serviceAgent.send(removeEquipletStepsAclMessage);
				
			} catch (UnreadableException e) {
				Logger.log(LogLevel.ERROR, "could not get the content object of the message", e);
			} catch (InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, "Error when removing the service steps", e);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
