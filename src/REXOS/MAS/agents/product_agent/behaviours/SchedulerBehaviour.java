/**
 * @file rexos/mas/productAgent/SchedulerBehaviour.java
 * @brief Behaviour in which the product agent schedules the productsteps.
 * @date Created: 23-04-2013
 * 
 * @author Ricky van Rijn
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package agents.product_agent.behaviours;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.BehaviourStatus;
import agents.data_classes.DbData;
import agents.data_classes.EquipletScheduleInformation;
import agents.data_classes.Product;
import agents.data_classes.Production;
import agents.data_classes.ProductionEquipletMapper;
import agents.data_classes.ProductionStep;
import agents.data_classes.StepStatusCode;
import agents.product_agent.BehaviourCallback;
import agents.product_agent.ProductAgent;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

import configuration.Configuration;
import configuration.ConfigurationFiles;

@SuppressWarnings("serial")
public class SchedulerBehaviour extends Behaviour {

	private ProductAgent productAgent;
	private ProductionStep prodStep;

	private boolean isError = false;
	private boolean isCompleted = false;

	private BehaviourCallback behaviourCallback;

	private int schedulersStarted = 0;
	private int schedulersCompleted = 0;
	
	private boolean scheduleInformationDone = false;
	
	private HashMap<AID, EquipletScheduleInformation> equipletSchedules = new HashMap<AID, EquipletScheduleInformation>();
	private ArrayList<AID> refusedEquiplets = new ArrayList<AID>();
	
	/**
	 * Construct scheudler behavior
	 * @param myAgent
	 * @param bc
	 */
	public SchedulerBehaviour(ProductAgent productAgent, BehaviourCallback bc) {
		super(productAgent);
		this.productAgent = productAgent;
		this.behaviourCallback = bc;
	}

	/**
	 * Performs the schedule algorithm
	 */
	@Override
	public void onStart() {
		try {
			ProductionEquipletMapper pem = productAgent.getProduct().getProduction()
					.getProductionEquipletMapping();
			
			// Shedule the PA with the equiplet agents in the current list.
			productAgent.getProduct().getProduction()
					.getProductionEquipletMapping();

			Product product = this.productAgent.getProduct();
			Production production = product.getProduction();
			ArrayList<ProductionStep> psa = production.getProductionSteps();
			
			//get a list of equiplets we need to get the schedule of
			ArrayList<AID> equipletSchedulesToGet = new ArrayList<AID>();
			for ( ProductionStep productStep : psa){
				int productStepId = productStep.getId();
				HashMap <AID, Long> equipletMapping = pem.getEquipletsForProductionStep(productStepId);
				
				for ( AID equiplet : equipletMapping.keySet()){
					if (! equipletSchedulesToGet.contains(equiplet)){
						equipletSchedulesToGet.add(equiplet);
					}
				}
			}

			productAgent.addBehaviour(new ScheduleInformationBehaviour(productAgent, this, equipletSchedulesToGet.toArray(new AID[equipletSchedulesToGet.size()]), this));
			//here we have the schedules of the equiplets
			//choose the right equiplet for the steps
			
			// Notify the OverviewBehaviour that the scheduler is running. The
			// overview behaviour will start the produceBehaviour so it's
			// possible to
			// schedule and produce at the same time.
			this.behaviourCallback.handleCallback(BehaviourStatus.RUNNING, null);

		/*	for (ProductionStep ps : psa) {
				Logger.log(LogLevel.DEBUG, "Trying to schedule a new step (id "
						+ ps.getId() + "). status: " + ps.getStatus());

				if ((ps.getStatus() == StepStatusCode.EVALUATING || ps
						.getStatus() == StepStatusCode.RESCHEDULE)
						&& isError == false) {
					int PS_Id = ps.getId();
					java.util.HashMap<AID, Long> equiplets = production.getProductionEquipletMapping().getEquipletsForProductionStep(PS_Id);

					if (equiplets != null && equiplets.size() != 0) 
					{
						Logger.log(LogLevel.INFORMATION, "Added scheduler");
						Scheduler(equiplets.keySet(), ps);
						schedulersStarted++;
					} 
					else 
					{
						Logger.log(LogLevel.ERROR, "Equiplets are null ( or size is 0 )");
						isError = true;
					}
				}
			}
		*/
		}
		catch (Exception e) 
		{
			e.printStackTrace();
			Logger.log(LogLevel.ERROR, e.toString());
		}
	}
	
	/**
	 * Sets the status of the behavior
	 */
	@Override
	public void action() {
		if (isError) {
				behaviourCallback.handleCallback(BehaviourStatus.ERROR, null);
			isCompleted = true;
		}
		
		//we need to be blocked waiting until we have the scheduleinformation
		if ( !scheduleInformationDone){
			block();
		}
		else if ( scheduleInformationDone){
			//we have our scheduleinformation
			//now start the logic to choose the equiplets
			
			//TODO: current logic is to choose the first equiplet. improve it.
			AID chosenEquiplet = equipletSchedules.keySet().iterator().next();
			
			
		}
	//	if ( schedulersStarted == schedulersCompleted && isError == false) {
	//		Logger.log(LogLevel.INFORMATION, "Setting scheduler to complete");
	//		behaviourCallback.handleCallback(BehaviourStatus.COMPLETED, null);
		//	isCompleted = true;
	//	} else if (isError) {
	//		behaviourCallback.handleCallback(BehaviourStatus.ERROR, null);
		//	isCompleted = true;
	//	}
	}

	/**
	 * Returns true when the behavior is done
	 * @return
	 */
	@Override
	public boolean done() {
		return isCompleted;
	}

	@Override
	public void reset() {
		super.reset();
	}
	
	@Override
	public void restart(){
		super.restart();
		isError = false;
        isCompleted = false;

        schedulersStarted = 0;
        schedulersCompleted = 0;
	}

	public void callbackScheduleInformation(HashMap<AID, EquipletScheduleInformation> equipletSchedules, ArrayList<AID> refusedEquiplets, 
											SchedulerBehaviour scheduleBehaviourThread){
		this.equipletSchedules = equipletSchedules;
		this.refusedEquiplets = refusedEquiplets;
		
		Logger.log(LogLevel.DEBUG, "ScheduleInformationBehaviour is done, continuing the ScheduleBehaviour");
		scheduleInformationDone = true;
		scheduleBehaviourThread.reset();
	}
	
	
	/**
	 * Scheduler function schedules the given production step
	 * 
	 * @param equipletList
	 * @param productionStep
	 * @throws Exception
	 */
	public void Scheduler(Set<AID> equipletList,
			final ProductionStep productionstep) {
		try {
			this.prodStep = productionstep;
	
			List<AID> equipletlist = new ArrayList<AID>(equipletList);
	
			BlackboardClient bbc = new BlackboardClient(
					Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"), 
					Integer.parseInt(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbPort")));
			
			bbc.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbName"));
			bbc.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "timeDataCollectionName"));
			
			BasicDBObject dbObject = (BasicDBObject) bbc.findDocuments(new BasicDBObject()).get(0);
			
			long firstTimeSlot = dbObject.getLong("firstTimeSlot");
			int timeSlotLength = dbObject.getInt("timeSlotLength");
	
			Logger.log(LogLevel.INFORMATION, "First Timeslot: " + firstTimeSlot + " timeslotLength: " + timeSlotLength);
			
			ArrayList<FreeTimeSlot> freetimeslots = new ArrayList<FreeTimeSlot>();
			DbData dbData = null;
			// Change this
			for (AID aid : equipletlist) 
			{
				Logger.log(LogLevel.INFORMATION, "Trying to reach equiplet: " + aid.getLocalName() + "");
				
				bbc = new BlackboardClient(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"));
				bbc.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbName"));
				bbc.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletDirectoryName"));
	
				QueryBuilder qb = QueryBuilder.start("AID").is(aid.getName());
	
				List<DBObject> aidInfo = bbc.findDocuments(qb.get());
	
				if (aidInfo.size() > 0) 
				{
					dbData = new DbData((BasicDBObject) aidInfo.get(0).get("db"));
				} 
				else 
				{
					Logger.log(LogLevel.ERROR, "There doesnt seem to be any equiplet available..");
				}
	
				ArrayList<Schedule> schedules = new ArrayList<Schedule>();
				ProductAgent prodAgent = (ProductAgent) myAgent;
	
				bbc = new BlackboardClient(dbData.getIp(), dbData.getPort());
				bbc.setDatabase(dbData.getName());
				bbc.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ProductStepsBlackBoardName", aid.getLocalName()));
	
				int requiredTimeSlots = (int) prodAgent.getProduct()
						.getProduction().getProductionEquipletMapping()
						.getTimeSlotsForEquiplet(prodStep.getId(), aid);
	
				// Gets planned steps TODO: improve query
				DBObject query = QueryBuilder.start("scheduleData.startTime")
						.greaterThan(-1).put("scheduleData.startTime")
						.get();
				
				BasicDBObject orderby = new BasicDBObject("scheduleData",
						new BasicDBObject("startTime", "1"));
				
				BasicDBObject findquery = new BasicDBObject("$query", query)
						.append("$orderby", orderby);
				
				List<DBObject> plannedSteps = bbc.findDocuments(findquery);
				
				Logger.log(LogLevel.INFORMATION, "Planned steps count: " + plannedSteps.size() + " requiredSlots: " + requiredTimeSlots);
				
				for (int i = 0; i < plannedSteps.size(); i++) 
				{
					long startTime = ((BasicDBObject) plannedSteps.get(i).get("scheduleData")).getLong("startTime");
					long duration = ((BasicDBObject) plannedSteps.get(i).get("scheduleData")).getLong("duration");
					schedules.add(new Schedule(startTime, duration, aid));
				}
	
				// check within every schedule of the 'schedules' array for free
				// timeslots and add them to the 'freetimeslot' array
				
				if (schedules.size() > 0) {
					for (int index = 0; index < schedules.size(); index++) {
						if (schedules.size() > (index + 1)) 
						{
							// if((schedules.get((index+1)).getStartTime() -
							// schedules.get(index).getDeadline()) >
							// requiredTimeSlots) {
							// freetimeslots.add(new
							// FreeTimeSlot(schedules.get(index).getDeadline(),
							// requiredTimeSlots, aid));
							// }
						} 
						else 
						{
							Schedule lastSchedule = schedules.get(index);
							freetimeslots.add(new FreeTimeSlot(lastSchedule.getDeadline(), requiredTimeSlots * timeSlotLength, aid));
							Logger.log(LogLevel.INFORMATION, "Adding new timeslot to freetimeslot start: " + lastSchedule.getDeadline() + " timeslots: " + requiredTimeSlots);
						}
					}
				} 
				else 
				{
					freetimeslots.add(new FreeTimeSlot((System.currentTimeMillis() - firstTimeSlot) / timeSlotLength + (1000 / timeSlotLength),
							requiredTimeSlots, aid));
					Logger.log(LogLevel.INFORMATION, "Adding new timeslot to freetimeslot start: " + (System.currentTimeMillis() - firstTimeSlot) / timeSlotLength + (1000 / timeSlotLength) + " timeslots: " + requiredTimeSlots + 
							" length: " + requiredTimeSlots);
				}
			}
			
			FreeTimeSlot freetimeslotEq = null;
	
			// calculate freetime slot and asign them to the above intialized values
			if (freetimeslots.size() > 0) 
			{
				for (FreeTimeSlot fts : freetimeslots)
				{
					if (freetimeslotEq == null) 
					{
						freetimeslotEq = fts;
					} 
					else 
					{
						Logger.log(LogLevel.ERROR, "FreeTimeSlotEq != null");
					}
				}
			}
	
			ACLMessage msg = new ACLMessage(ACLMessage.REQUEST);
			msg.setConversationId(this.prodStep.getConversationIdForEquiplet(freetimeslotEq.getEquipletName()));
			msg.setOntology("ScheduleStep");
			msg.setContentObject(freetimeslotEq.getStartTime());
			msg.addReceiver(freetimeslotEq.getEquipletName());
			myAgent.send(msg);

			ACLMessage returnMsg = myAgent.blockingReceive(MessageTemplate.MatchOntology("Planned"), 10000);
			if (returnMsg != null){
				if (returnMsg.getPerformative() == ACLMessage.CONFIRM) {
					prodStep.setStatus(StepStatusCode.PLANNED);
					prodStep.setUsedEquiplet(returnMsg.getSender());
				} else if (returnMsg.getPerformative() == ACLMessage.DISCONFIRM) {
					isError = true;
					behaviourCallback.handleCallback(BehaviourStatus.ERROR, null);
				}	
			}
			else{
				//TODO: error handling
			}
			prodStep.setConversationId(returnMsg.getConversationId());
			schedulersCompleted++;

		} catch (InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "Database exception at scheduling", e);
		} catch (IOException e1){
			Logger.log(LogLevel.ERROR, "Message content exception at scheduling", e1);
		}
	}

	private class FreeTimeSlot {
		private long startTimeSlot = -1;
		private long duration = -1;
		private AID equipletName = null;

		/**
		 * Construct free time slot
		 * @param startTime
		 * @param duration
		 * @param equipletName
		 */
		public FreeTimeSlot(long startTime, long duration, AID equipletName) {
			this.startTimeSlot = startTime;
			this.duration = duration;
			this.equipletName = equipletName;
		}

		/**
		 * Gets the equiplet name
		 * @return
		 */
		public AID getEquipletName() {
			return this.equipletName;
		}

		/**
		 * Gets the start time of timeslot
		 * @return
		 */
		public long getStartTime() {
			return this.startTimeSlot;
		}

		/**
		 * Gets the duration of the timeslot
		 * @return
		 */
		public long getDuration() {
			return this.duration;
		}

		/**
		 * ToString annotation
		 */
		@Override
		public String toString() {
			return "{Start TimeSlot: " + this.startTimeSlot + ", Duration: "
					+ this.duration + ", EquipletName: " + this.equipletName
					+ "}";
		}
	}

	private class Schedule {
		private long startTime = -1;
		private long duration = -1;
		private long deadline = -1;
		private AID equipletName;

		/**
		 * Constructs Schedule object
		 * @param startTime
		 * @param duration
		 * @param equipletName
		 */
		public Schedule(long startTime, long duration, AID equipletName) {
			this.startTime = startTime;
			this.duration = duration;
			this.deadline = startTime + duration + 1;
			this.equipletName = equipletName;
		}

		/**
		 * get starting time
		 * @return
		 */
		public long getStartTime() {
			return this.startTime;
		}

		/**
		 * Set starting time
		 * @param newStartTime
		 */
		@SuppressWarnings("unused")
		public void setStartTime(int newStartTime) {
			this.startTime = newStartTime;
		}

		/**
		 * Gets equiplet name
		 * @return
		 */
		public AID getEquipletName() {
			return this.equipletName;
		}
		
		/**
		 * Gets deadline time
		 * @return
		 */
		public long getDeadline() {
			return this.deadline;
		}

		/**
		 * Sets the deadline time
		 * @param newDeadline
		 */
		@SuppressWarnings("unused")
		public void setDeadline(int newDeadline) {
			this.deadline = newDeadline;
		}

		/**
		 * ToString Annotation
		 */
		@Override
		public String toString() {
			return "{ startTime:" + startTime + ", duration:" + duration
					+ ", deadline:" + deadline + ", EquipletName:"
					+ equipletName + getEquipletName() + " }";
		}
	}
}
