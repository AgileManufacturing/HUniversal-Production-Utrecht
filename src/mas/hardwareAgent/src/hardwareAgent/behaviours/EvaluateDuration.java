package hardwareAgent.behaviours;

import java.sql.ResultSet;
import java.sql.SQLException;

import hardwareAgent.EquipletStepMessage;
import hardwareAgent.HardwareAgent;
import hardwareAgent.Module;
import hardwareAgent.TimeData;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import rexos.libraries.knowledge.KnowledgeDBClient;
import rexos.libraries.knowledge.Queries;
import rexos.libraries.knowledge.Row;
import serviceAgent.ServiceStepMessage;

import behaviours.ReceiveBehaviour;

import com.mongodb.BasicDBObject;

import newDataClasses.*;

public class EvaluateDuration extends ReceiveBehaviour {
	private static final long serialVersionUID = 1L;

	private static MessageTemplate messageTemplate = MessageTemplate.MatchOntology("GetServiceStepDuration");
	private HardwareAgent hardwareAgent;
	/**
	 * Authors: Thierry Gerritse 
	 * Class: EvaluateDuration.java* 
	 */

	/**
	 * Instantiates a new schedule step.
	 * 
	 * @param a
	 *            the a
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
			BasicDBObject parameters = (BasicDBObject) serviceStep.get("parameters");
			
			String serviceName = serviceStep.getString("serviceName");
						
			/**
			 * haal de naam van de leidende module uit knowledge db aan de hand van de servicestep.service (ofzo)
			 * 
			 * String moduleName = 
			 * 
			 */
			
			Module module = hardwareAgent.GetModuleByName("gripper");		
			
			EquipletStepMessage[] equipletSteps = module.getEquipletSteps(parameters);
			
			long stepDuration = 0l;
			
			for(EquipletStepMessage equipletStep : equipletSteps){
				
				TimeData td = equipletStep.timeData;
				stepDuration += td.getDuration();
				
			}
			
			ScheduleData schedule = new ScheduleData();
			schedule.fillWithBasicDBObject(((BasicDBObject) serviceStep.get("scheduleData")));
			schedule.setDuration(stepDuration);

			hardwareAgent.getServiceStepsBBClient().updateDocuments(
					new BasicDBObject("_id", objectId), 
					new BasicDBObject("$set", schedule.toBasicDBObject()));
			// plaats equipletsteps en hun duration en zijn status op eveluating
			// op bb,

			ACLMessage reply;
			reply = message.createReply();
			reply.setContentObject(objectId);
			reply.setOntology("GetServiceStepDurationResponse");
			myAgent.send(reply);

			// zet duration van de betreffende service step

			// stuur peter een reactie met het staat er
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
