package agents.hardware_agent.behaviours;

import org.bson.types.ObjectId;

import agents.hardware_agent.HardwareAgent;
import agents.service_agent.ServiceAgent;
import agents.shared_behaviours.ReceiveBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

public class RemoveEquipletStepBehaviour extends ReceiveBehaviour{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 681033710946072342L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      The messageTemplate to match the messages.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("RemoveEquipletStep");
	
	private HardwareAgent hardwareAgent;
	
	public RemoveEquipletStepBehaviour(HardwareAgent hardwareAgent) {
		super(hardwareAgent, MESSAGE_TEMPLATE);
		this.hardwareAgent = hardwareAgent;
	}

	@Override
	public void handle(ACLMessage message) {
		if (message.getPerformative() == ACLMessage.INFORM){
			try {
				ObjectId[] serviceStepIds = (ObjectId[]) message.getContentObject();
				for ( ObjectId serviceStepId : serviceStepIds){
					hardwareAgent.removeEquipletStepsByServiceStepId(serviceStepId);
				}
			} catch (UnreadableException e) {
				Logger.log(LogLevel.ERROR, "could not get the content object of the message", e);
			} catch (InvalidDBNamespaceException | GeneralMongoException e) {
				Logger.log(LogLevel.ERROR, "Error when removing the equiplet steps", e);
				e.printStackTrace();
			}
		}
	}
}
