/**
 * @file rexos/mas/productAgent/SchedulerBehaviour.java
 * @brief Behaviour in which the product agent schedules the productsteps.
 * @date Created: 23-04-2013
 * 
 * @author Ricky van Rijn
 * @author Mike Schaap
 * @author Alexander Streng
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
import jade.core.behaviours.Behaviour;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import libraries.schedule.data_classes.EquipletScheduleInformation;
import libraries.schedule.data_classes.FreeTimeSlot;
import libraries.schedule.data_classes.ProductStepSchedule;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.BehaviourStatus;
import agents.data_classes.Matrix;
import agents.data_classes.Product;
import agents.data_classes.ProductStepScheduleInformation;
import agents.data_classes.Production;
import agents.data_classes.ProductionEquipletMapper;
import agents.data_classes.ProductionStep;
import agents.product_agent.BehaviourCallback;
import agents.product_agent.ProductAgent;

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
	
	private LinkedHashMap<AID, EquipletScheduleInformation> equipletSchedules = new LinkedHashMap<AID, EquipletScheduleInformation>();

	private ArrayList<ProductionStep> productionSteps;
	/**
	 * Construct scheduler behavior
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
			Product product = productAgent.getProduct();
			Production production = product.getProduction();
			productionSteps = production.getProductionSteps();
			
			//get a list of equiplets we need to get the schedule of
			ArrayList<AID> equipletSchedulesToGet = new ArrayList<AID>();
			for (ProductionStep productStep : productionSteps){
				int productStepId = productStep.getId();
				HashMap <AID, Long> equipletMapping = pem.getEquipletsForProductionStep(productStepId);
				
				for (AID equiplet : equipletMapping.keySet()) {
					if (!equipletSchedulesToGet.contains(equiplet)){
						equipletSchedulesToGet.add(equiplet);
					}
				}
			}

			productAgent.addBehaviour(new ScheduleInformationBehaviour(productAgent, this, equipletSchedulesToGet.toArray(new AID[equipletSchedulesToGet.size()])));

			this.behaviourCallback.handleCallback(BehaviourStatus.RUNNING, null);
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
		
		//we need to wait until we have the scheduleinformation
		if (!scheduleInformationDone){
			block();
		} else if (scheduleInformationDone){
			//Generate the scheduleMatrix
			Matrix scheduleMatrix = generateScheduleMatrix();
			
			//start scheduling
			if(schedule(scheduleMatrix)){
				//scheduling is done
				//smth like set callback behaviour
			} else {
				//Need to reschedule?
			}
		}
	}

	/**
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more information see 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user requirements to product' - Leo van Moergestel section 3.2 
	 * @return the generated scheduleMatrix
	 */
	private Matrix generateScheduleMatrix() {
		ProductionEquipletMapper equipletMapper = productAgent.getProduct().getProduction().getProductionEquipletMapping();
		
		//construct a new matrices to perform a neat-o selection
		Matrix scheduleMatrix = new Matrix(equipletSchedules.size(), productionSteps.size());
		
		int row = 0, column = 0, sequenceLength, firstInSequence;
			//Iterate through them steps to fill the matrix
			for (AID equiplet : equipletSchedules.keySet()) { // row
				sequenceLength = 0; firstInSequence = -1; // always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
				for (ProductionStep productStep : productionSteps) { // column
				
				double canPerformStepValue = (equipletMapper.canEquipletPerformProductStep(productStep.getId(), equiplet)) ? 1.0 : 0.0;
			
				if(canPerformStepValue == 1.0) {   //increase sequence counter.
					if(firstInSequence < 0){	  //set the first item in the sequence.
						firstInSequence = column;
					}
					sequenceLength++;
					if(column == productionSteps.size()){ // end of row
						setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
					}
				} else if(canPerformStepValue == 0.0 && sequenceLength > 0) { // end of sequence
					setSequenceValues(row, firstInSequence, sequenceLength, scheduleMatrix);
					sequenceLength = 0;
					firstInSequence = -1;
				}
				
				//value might have changed since we added sequence multiplier
				double loadValue = ((EquipletScheduleInformation)equipletSchedules.get(equiplet)).getLoad();
				
				//TODO: perform supermagic calculation of currentValue * load here
				scheduleMatrix.set(row, column, (scheduleMatrix.get(row, column) * loadValue));
				column++;
			}
			row++;
		}
		return scheduleMatrix;
	}
	
	
	@SuppressWarnings("unchecked")
	private boolean schedule(Matrix scheduleMatrix) {
		ArrayList<ProductStepScheduleInformation> finalSchedules = new ArrayList<ProductStepScheduleInformation>();
		
		// Read the matrix. Write function to iterate each seperate row ( productsteps ) and pick each equiplet 
		for (int column = 0; column < scheduleMatrix.getNumberOfColumns(); column++) { //Productsteps 
			int highestEquipletScoreIndex = -1;
			ProductionStep productionStep = productionSteps.get(column);
			
			for (int row = 0; row < scheduleMatrix.getNumberOfRows(); row++) { //AID'S
				highestEquipletScoreIndex = (scheduleMatrix.get(row, column) > scheduleMatrix.get(highestEquipletScoreIndex, column)) ? row : highestEquipletScoreIndex;
			}
			
			if(highestEquipletScoreIndex < 0){
				Logger.log(LogLevel.ERROR, "No suitable equiplet found for this step! Scheduling has gone wrong.. Reschedule?");
				return false;
			}
			
			//Can we assume that all productSteps are ordered? What about parallel steps?
			EquipletScheduleInformation scheduleInformation = ((ArrayList<EquipletScheduleInformation>)equipletSchedules.values()).get(highestEquipletScoreIndex);
			AID equipletId = ((ArrayList<AID>)equipletSchedules.keySet()).get(highestEquipletScoreIndex);
			
			//Get first free timeslot
			FreeTimeSlot freeTimeSlot = scheduleInformation.getFreeTimeSlots(productionStep.getDurationForEquiplet(equipletId)).get(0);
			
			//Check the equiplets schedule. Lets check if the schedule fits. Keep in mind that the deadline is met.
			if(scheduleInformation.getIsEquipletScheduleLocked() && freeTimeSlot != null) {
				
				//Get first free timeslot, and make an list (array?)
				finalSchedules.add(new ProductStepScheduleInformation(productionStep, equipletId, 
						new ProductStepSchedule(productionStep.getConversationId(), freeTimeSlot.getTimeSlot(), 5l)));
				
				//plan the timeslot
				scheduleInformation.planTimeSlot(freeTimeSlot);
			}
			
			// If the schedule fits, save the equiplet with corresponding step(s) ( maybe equipletmapper? )
		}
		// Message all the equiplets with their correspondig equiplet steps
		
		
		return false;
	}
	/**
	 * Sets the value of the sequences. @ref to paper Multiagent-based agile manufacturing: from user requirements to product leo van moergestel
	 * section 3.2 
	 */
	private void setSequenceValues(int row, int firstInSequence, int sequenceLength, Matrix matrix){
		int value = sequenceLength -1;
		for(int i = firstInSequence; i <= sequenceLength; i++){
			matrix.set(row, i, (matrix.get(row, i) + value));
		}
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
		isError = false;
        isCompleted = false;

        schedulersStarted = 0;
        schedulersCompleted = 0;
	}
	
	@Override
	public void restart(){
		super.restart();
	}

	public void callbackScheduleInformation(LinkedHashMap<AID, EquipletScheduleInformation> equipletSchedules, SchedulerBehaviour schedulerBehaviour){
		schedulerBehaviour.equipletSchedules = equipletSchedules;		
		Logger.log(LogLevel.DEBUG, "ScheduleInformationBehaviour is done, continuing the ScheduleBehaviour");
		scheduleInformationDone = true;
		restart();
	}
}
	/*
	public 
	/**
	 * Scheduler function schedules the given production step
	 * 
	 * @param equipletList
	 * @param productionStep
	 * @throws Exception
	 */
	/*
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
	} */