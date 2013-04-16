package hardwareAgent.behaviours;

import hardwareAgent.HardwareAgent;
import hardwareAgent.Module;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class EvaluateDuration extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	
	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("GetServiceStepDuration");
	private HardwareAgent hardwareAgent;

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a the a
	 */
	public EvaluateDuration(Agent a) {
		super(a, -1, messageTemplate);
		hardwareAgent = (HardwareAgent) a;
	}

	@Override
	public void handle(ACLMessage message) {
		Object contentObject = null;
		String contentString = message.getContent();

		try {
			contentObject = message.getContentObject();
		} catch (UnreadableException e) {
			// System.out.println("Exception Caught, No Content Object Given");
		}
		System.out.format("%s received message from %s (%s:%s)%n", myAgent.getLocalName(), message.getSender().getLocalName(), message.getOntology(), contentObject == null ? contentString : contentObject);

		try {
			ObjectId objectId = null;
			BasicDBObject serviceStep = null;
			try {
				objectId = (ObjectId) message.getContentObject();
				serviceStep = (BasicDBObject) hardwareAgent.getServiceStepsBBClient().findDocumentById(objectId);
			} catch (UnreadableException | InvalidDBNamespaceException e) {
				e.printStackTrace();
				myAgent.doDelete();
			}

			long stepType = serviceStep.getLong("type");
			Object parameters = serviceStep.get("parameters");
			
			// kijk in hashmap welke module hoort bij deze step
			
			Module leadingModule = (Module)hardwareAgent.GetModuleForStep(stepType);
			
			//vraag aan die module vertaal deze service step in equipletsteps
			
			long stepDuration = leadingModule.getStepDuration();
			
			//plaats equipletsteps en hun duration en zijn status op eveluating op bb,
			
			ACLMessage reply;
			reply = message.createReply();
			reply.setContent("it is there!");
			myAgent.send(reply);
			
			// zet duration van de betreffende service step
			
			
			// stuur peter een reactie met het staat er
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private boolean PickUpTranslation(Object parameters) {
		// double x = parameters.getX();
		// double y = parameters.getY();
		// double z = parameters.getZ();
		// long relative = parameters.getRelative();

		// Move delta robot to *,*,8 relative to crate A(relative)
		// Move delta robot to x,y,* relative to crate A(relative)
		// Move delta robot to *,*,z relative to crate A(relative)
		// Activate gripper
		// Move delta robot to *,*,8 relative to crate A(relative)
		return true;
	}

	private boolean DropTranslation(Object parameters) {
		// double x = parameters.getX();
		// double y = parameters.getY();
		// double z = parameters.getZ();
		// long relative = parameters.getRelative();

		// Move delta robot to *,*,8 relative to crate A
		// Move delta robot to x,y,* relative to crate A
		// Move delta robot to *,*,z relative to crate A
		// Deactivate gripper
		// Move delta robot to *,*,8 relative to crate A
		return true;
	}
}
