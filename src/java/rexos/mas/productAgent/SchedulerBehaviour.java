/**
 * @file SchedulerBehaviour.java
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

package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.log.Logger;
import rexos.mas.data.BehaviourStatus;
import rexos.mas.data.DbData;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

@SuppressWarnings("serial")
public class SchedulerBehaviour extends Behaviour {

	private ProductAgent _productAgent;
	private ProductionStep _prodStep;

	private boolean _isError = false;
	private boolean _isCompleted = false;

	private BehaviourCallback _bc;

	private int _schedulersStarted = 0;
	private int _schedulersCompleted = 0;

	public SchedulerBehaviour(Agent myAgent, BehaviourCallback bc) {
		super(myAgent);
		this._bc = bc;
	}

	@Override
	public void onStart() {
		try {
			// Shedule the PA with the equiplet agents in the current list.
			_productAgent = (ProductAgent) myAgent;
			_productAgent.getProduct().getProduction().getProductionEquipletMapping();

			Product product = this._productAgent.getProduct();
			Production production = product.getProduction();
			ArrayList<ProductionStep> psa = production.getProductionSteps();

			for (ProductionStep ps : psa) {
				int PA_id = ps.getId();
				java.util.HashMap<AID, Long> equiplets = production.getProductionEquipletMapping().getEquipletsForProductionStep(PA_id);

				if (equiplets != null && equiplets.size() != 0) {

					Scheduler(production.getProductionEquipletMapping().getEquipletsForProductionStep(PA_id).keySet(), ps);
					_schedulersStarted++;
				} else {
					_isError = true;
				}

			}
		} catch (Exception e) {
			e.printStackTrace();
			Logger.log(e);
		}
	}

	@Override
	public void action() {
		try {
			if (_schedulersStarted == _schedulersCompleted) {
				this._bc.handleCallback(BehaviourStatus.COMPLETED, null);
				_isCompleted = true;
			} else if (_isError) {
				this._bc.handleCallback(BehaviourStatus.ERROR, null);
				_isCompleted = true;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public boolean done() {
		return _isCompleted;
	}

	/**
	 * Scheduler function schedules the given production step
	 * 
	 * @param equipletList
	 * @param productionStep
	 * @throws Exception
	 */
	public void Scheduler(Set<AID> equipletList, final ProductionStep productionstep) throws Exception {

		this._prodStep = productionstep;

		// load set into arraylist
		List<AID> equipletlist = new ArrayList<AID>(equipletList);
		//Create Hashmap for database data
		//HashMap<AID, DbData> dbData = new HashMap<AID, DbData>();
		
		BlackboardClient bbc = new BlackboardClient("145.89.191.131");
		bbc.setDatabase("CollectiveDb");
		bbc.setCollection("TimeData");
		BasicDBObject dbObject = (BasicDBObject)bbc.findDocuments(new BasicDBObject()).get(0);
		long firstTimeSlot = dbObject.getLong("firstTimeSlot");
		int timeSlotLength = dbObject.getInt("timeSlotLength");


		ArrayList<FreeTimeSlot> freetimeslots = new ArrayList<FreeTimeSlot>();
		DbData dbData = null;
		// Change this
		for (AID aid : equipletlist) {
			bbc = new BlackboardClient("145.89.191.131");
			bbc.setDatabase("CollectiveDb");
			bbc.setCollection("EquipletDirectory");

			QueryBuilder qb = QueryBuilder.start("AID").is(aid.getName());

			List<DBObject> aidInfo = bbc.findDocuments(qb.get());
			
			if (aidInfo.size() > 0) {
				dbData = new DbData((BasicDBObject) aidInfo.get(0).get("db"));
			} else {
				// TODO: what to do if list is empty
			}
			
			ArrayList<Schedule> schedules = new ArrayList<Schedule>();
			ProductAgent prodAgent = (ProductAgent) myAgent;

			bbc = new BlackboardClient(dbData.getIp(), dbData.getPort());
			bbc.setDatabase(dbData.getName());
			bbc.setCollection("ProductStepsBlackBoard");
			
			int requiredTimeSlots = (int) prodAgent.getProduct().getProduction().getProductionEquipletMapping().getTimeSlotsForEquiplet(_prodStep.getId(), aid);

			// Gets planned steps TODO:: improve query
			DBObject query = QueryBuilder.start("scheduleData.startTime").greaterThan(-1).put("scheduleData.startTime").greaterThan(System.currentTimeMillis()/timeSlotLength).get();
			BasicDBObject orderby = new BasicDBObject("scheduleData", new BasicDBObject("startTime", "1"));
			BasicDBObject findquery = new BasicDBObject("$query", query).append("$orderby", orderby);
			List<DBObject> plannedSteps = bbc.findDocuments(findquery);
			for (int i = 0; i < plannedSteps.size(); i++) {
				long startTime = ((BasicDBObject) plannedSteps.get(i).get("scheduleData")).getLong("startTime");
				int duration = ((BasicDBObject) plannedSteps.get(i).get("scheduleData")).getInt("duration");
				schedules.add(new Schedule(startTime, duration, aid));
			}

			// check within every schedule of the 'schedules' array for free
			// timeslots and add them to the 'freetimeslot' array
			if (schedules.size() > 0) {
				for (int index = 0; index < schedules.size(); index++) {
					if (schedules.size() > (index + 1)) {
						if((schedules.get((index+1)).getStartTime() - schedules.get(index).getDeadline()) > requiredTimeSlots) {
							freetimeslots.add(new FreeTimeSlot(schedules.get(index).getDeadline(), requiredTimeSlots, aid));
						}
					} else {
						Schedule lastSchedule = schedules.get(index);
						freetimeslots.add(new FreeTimeSlot(lastSchedule.getDeadline() + 50, requiredTimeSlots, aid));
					}
				}
			} else {
				freetimeslots.add(new FreeTimeSlot((System.currentTimeMillis() - firstTimeSlot) / timeSlotLength + (5000/timeSlotLength), requiredTimeSlots, aid));
			}
		}
		FreeTimeSlot freetimeslotEq = null;

		// calculate freetime slot and asign them to the above intialized values
		if (freetimeslots.size() > 0) {
			for (FreeTimeSlot fts : freetimeslots) {
				if (freetimeslotEq == null || freetimeslotEq.getDuration() > fts.getDuration()) {
					freetimeslotEq = fts;
				}
			}
		}
		
		ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
		msg.setConversationId(this._prodStep.getConversationIdForEquiplet(freetimeslotEq.getEquipletName()));
		msg.setOntology("ScheduleStep");
		msg.setContentObject(freetimeslotEq.getStartTime());
		msg.addReceiver(freetimeslotEq.getEquipletName());
		myAgent.send(msg);
		
		ACLMessage returnMsg = myAgent.blockingReceive();
		if (returnMsg.getPerformative() == ACLMessage.CONFIRM) {
			_prodStep.setStatus(StepStatusCode.PLANNED);
			System.out.println("Planned");
		} else if (returnMsg.getPerformative() == ACLMessage.DISCONFIRM) {
			System.out.println("Disconfirm.");
		}
		_schedulersCompleted++;
		System.out.println("received message");
	}

	private class FreeTimeSlot {
		private long startTimeSlot = -1;
		private long duration = -1;
		private AID equipletName = null;

		public FreeTimeSlot(long start, int dura, AID equiplet) {
			this.startTimeSlot = start;
			this.duration = dura;
			this.equipletName = equiplet;
		}

		public AID getEquipletName() {
			return this.equipletName;
		}

		public long getStartTime() {
			return this.startTimeSlot;
		}

		public long getDuration() {
			return this.duration;
		}

		@Override
		public String toString() {
			return "{Start TimeSlot: " + this.startTimeSlot + ", Duration: " + this.duration + ", EquipletName: " + this.equipletName + "}";
		}
	}

	private class Schedule {
		private long startTime = -1;
		private int duration = -1;
		private long deadline = -1;
		private AID equipletName;

		public Schedule(long start, int dura, AID equiplet) {
			this.startTime = start;
			this.duration = dura;
			this.deadline = start + dura + 1;
			this.equipletName = equiplet;
		}

		public long getStartTime() {
			return this.startTime;
		}

		@SuppressWarnings("unused")
		public void setStartTime(int newStartTime) {
			this.startTime = newStartTime;
		}

		public AID getEquipletName() {
			return this.equipletName;
		}

		public long getDeadline() {
			return this.deadline;
		}

		@SuppressWarnings("unused")
		public void setDeadline(int newDeadline) {
			this.deadline = newDeadline;
		}

		@Override
		public String toString() {
			return "{ startTime:" + startTime + ", duration:" + duration + ", deadline:" + deadline + ", EquipletName:" + equipletName + getEquipletName() +" }";
		}
	}
}
