/**
 * @file SchedulerBehaviour.java
 * @brief Behaviour in which the product agent schedules the productsteps.
 * @date Created: 23-04-2013
 * 
 * @author Ricky van Rijn
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

package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.log.Logger;
import rexos.mas.data.DbData;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

@SuppressWarnings("serial")
public class SchedulerBehaviour extends OneShotBehaviour{
	private ProductAgent _productAgent;
	private int timeslotsToSchedule = 0;
	private int debug = 1;
	private ProductionStep _prodStep;

	@Override
	public void action(){
		// Shedule the PA with the equiplet agents in the current list.
		_productAgent = (ProductAgent) myAgent;
		_productAgent.getProduct().getProduction()
				.getProductionEquipletMapping();
		try{
			Product product = this._productAgent.getProduct();
			Production production = product.getProduction();
			ArrayList<ProductionStep> psa = production.getProductionSteps();
			if (debug != 0){
				// debug
				System.out.println("Number of equiplets: " + psa.size());
			}
			for(ProductionStep ps : psa){
				int PA_id = ps.getId();
				java.util.HashMap<AID, Long> equiplets = production.getProductionEquipletMapping()
						.getEquipletsForProductionStep(PA_id);
				if (equiplets != null && equiplets.keySet().size() > 0){
					this.timeslotsToSchedule = production
							.getProductionEquipletMapping()
							.getTimeSlotsForEquiplet(
									PA_id,
									(AID) production
											.getProductionEquipletMapping()
											.getEquipletsForProductionStep(
													PA_id).keySet().toArray()[0])
							.intValue();
				}
				if (debug != 0){
					// debug
					System.out.println("-------------------");
					System.out.println("step_id:"
							+ PA_id
							+ " number of eq available: "
							+ (equiplets != null ? equiplets.keySet().size() : 0));
					System.out.println("STEP_ID:" + ps.getId() + " requires "
							+ this.timeslotsToSchedule + " timeslots");
					System.out.println("-------------------");
				}
				if (equiplets != null && equiplets.size() != 0)
				{
					Scheduler(production.getProductionEquipletMapping().getEquipletsForProductionStep(PA_id).keySet(), ps);

				}
						
			}
		} catch(Exception e){
			Logger.log(e);
		}
	}

	/**
	 * Scheduler function schedules the given production step
	 * 
	 * @param equipletList
	 * @param productionStep
	 * @throws Exception
	 */
	public void Scheduler(Set<AID> equipletList, final ProductionStep productionstep)
			throws Exception{
		
		this._prodStep = productionstep;
		
		// load set into arraylist
		List<AID> equipletlist = new ArrayList<AID>(equipletList);
		
		HashMap<AID, DbData> dbData = new HashMap<AID, DbData>(); 
		
		//Change this
		
		for(AID aid : equipletlist) {
			
			BlackboardClient bbc = new BlackboardClient("145.89.191.131");
			bbc.setDatabase("CollectiveDb");
			bbc.setCollection("EquipletDirectory");
			
			QueryBuilder qb = QueryBuilder.start("AID").is(aid.getName());
			
			List<DBObject> aidInfo = bbc.findDocuments(qb.get());
			
			if(aidInfo.size() > 0) {
				dbData.put(aid, new DbData((BasicDBObject) aidInfo.get(0).get("db")));
			} else {
				// TODO: what to do if list is empty
			}	
		}
		
		int scheduleCount = 0;
		Schedule[] schedules;
		ArrayList<FreeTimeSlot> freetimeslot = new ArrayList<FreeTimeSlot>();
		
	    Iterator<Entry<AID, DbData>> it = dbData.entrySet().iterator();
	    while (it.hasNext()) {
	        Map.Entry<AID, DbData> pairs = it.next();
	        
	        DbData dbDa = pairs.getValue();
	        
			BlackboardClient bbc = new BlackboardClient(dbDa.getIp(), dbDa.getPort());
			bbc.setDatabase(dbDa.getName());
			bbc.setCollection("ProductStepsBlackboard");
			
			List<DBObject> blackBoard = bbc.findDocuments(" ");
			scheduleCount = blackBoard.size();
			
			schedules = new Schedule[scheduleCount];
			
			
			// Gets planned steps
			List<DBObject> plannedSteps =
								bbc.findDocuments(QueryBuilder.start("scheduleData.startTime").greaterThan(-1).get());
			for(int i = 0; i < plannedSteps.size(); i++){
				double b = (Double) plannedSteps.get(i).get("startTime");
				int stati = (int) b;
				double c = (Double) plannedSteps.get(i).get("duration");
				int dur = (int) c;
				// add scheduled timeslot to array of scheduled timeslots and
				// mention which equiplet
				schedules[i] = new Schedule(stati, dur, pairs.getKey());
			}
			
			int startTimeSlot = 0;
			// check within every schedule of the 'schedules' array for free
			// timeslots
			// and add them to the 'freetimeslot' array
			for(int run = 0; run < schedules.length; run++){
				if (schedules[run].getStartTime() > startTimeSlot){
					if (schedules.length > (run + 1)){
						if (schedules[run].getDeadline() < schedules[(run + 1)]
								.getStartTime()){
							int freeTimeSlot = schedules[(run + 1)].getStartTime()
									- schedules[run].getDeadline() - 1;
							int timeslotToSchedule = (schedules[run].getDeadline() + 1);
							freetimeslot.add(new FreeTimeSlot(
									timeslotToSchedule, freeTimeSlot,
									schedules[run].getEquipletName()));
							if (debug != 0){
								// debug
								System.out.println("Free timeslot: " + freeTimeSlot
										+ " starting at timeslot: "
										+ timeslotToSchedule);
							}
						}
					}
				}
			}
			if (schedules.length == 0) {
				freetimeslot.add(new FreeTimeSlot((int)(System.currentTimeMillis() / 2000 + 5), productionstep.getRequiredTimeSlots(), pairs.getKey()));
			}
			
	    }
	    
	    FreeTimeSlot freetimeslotEq = null;
		if (debug != 0){
			System.out.println("---- Number of timeslots to schedule -----");
			System.out.println("Timeslots to schedule: " + timeslotsToSchedule);
			System.out.println();
		}
		
		// calculate freetime slot and asign them to the above intialized values
		if (freetimeslot.size() > 0){
			if (debug != 0){
				System.out.println("Free time slots:" + freetimeslot.size());
			}
			for(FreeTimeSlot fts : freetimeslot) {
				if (fts.getDuration() <= timeslotsToSchedule){
					freetimeslotEq = fts;
				}
			}
		}
		
		if (freetimeslotEq != null && freetimeslotEq.getEquipletName() != null){
			ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
			msg.setConversationId(this._prodStep.getConversationId());
			msg.setOntology("ScheduleStep");
			msg.setContentObject(freetimeslotEq.getStartTime());
			msg.addReceiver(freetimeslotEq.getEquipletName());
			myAgent.send(msg);
			/*
			 * SEND MESSAGES TO OTHER PLATFORMS computername is resolved by the
			 * hosts file in either Linux or Windows AID remoteAMS = new
			 * AID("agentname@"+containername, AID.ISGUID);
			 * remoteAMS.addAddresses
			 * ("http://"+computername+":"+portnumber+"/acc");
			 * msg.addReceiver(remoteAMS); myAgent.send(msg);
			 */
			if (debug != 0){
				// debug
				System.out.println("Send timeslot " + freetimeslotEq.getStartTime()
						+ " to EQ name is "+freetimeslotEq.getEquipletName());
			}
		} else{
			if (debug != 0){
				// debug
				System.out.println("No timeslot asigned.");
			}
		}
		
		final MessageTemplate msgtemplate = MessageTemplate.and(
				MessageTemplate.MatchConversationId(this._prodStep.getConversationId()),
				MessageTemplate.MatchOntology("Planned"));
		
		
		
		((SequentialBehaviour) parent).addSubBehaviour(new ReceiveBehaviour(myAgent,
				10000, msgtemplate){
			/**
					 * 
					 */
			private static final long serialVersionUID = 1L;
			private boolean debug = true;

			@Override
			public void handle(
					ACLMessage msg){
				if (msg == null){
						System.out.println("Null message");
				} else{
					productionstep.setStatus(StepStatusCode.PLANNED);
					System.out.println("Add a ProduceBehaviour");
					ProduceBehaviour _produceBehaviour = new ProduceBehaviour(myAgent);
					((SequentialBehaviour) parent).addSubBehaviour(_produceBehaviour);
				}
			}
		});

		
	}

	private class FreeTimeSlot{
		private int startTime = -1;
		private int duration = -1;
		private AID equipletName = null;

		public FreeTimeSlot(int start, int dura, AID equiplet){
			this.startTime = start;
			this.duration = dura;
			this.equipletName = equiplet;
		}

		public AID getEquipletName(){
			return this.equipletName;
		}

		public int getStartTime(){
			return this.startTime;
		}

		public int getDuration(){
			return this.duration;
		}

		@Override
		public String toString(){
			return "{Start TimeSlot: " + this.startTime + ", Duration: "
					+ this.duration + ", EquipletName: " + this.equipletName
					+ "}";
		}
	}

	private class Schedule{
		private int startTime = -1;
		private int duration = -1;
		private int deadline = -1;
		private AID equipletName;

		public Schedule(int start, int dura, AID equiplet){
			this.startTime = start;
			this.duration = dura;
			this.deadline = start + dura - 1;
			this.equipletName = equiplet;
		}

		public int getStartTime(){
			return this.startTime;
		}

		@SuppressWarnings("unused")
		public void setStartTime(int newStartTime){
			this.startTime = newStartTime;
		}

		public AID getEquipletName(){
			return this.equipletName;
		}

		public int getDeadline(){
			return this.deadline;
		}

		@SuppressWarnings("unused")
		public void setDeadline(int newDeadline){
			this.deadline = newDeadline;
		}

		@Override
		public String toString(){
			return "{ startTime:" + startTime + ", duration:" + duration
					+ ", deadline:" + deadline + ", EquipletName:"
					+ equipletName + " }";
		}
	}
}
