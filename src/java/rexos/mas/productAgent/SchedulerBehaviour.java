package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

import rexos.mas.newDataClasses.Product;
import rexos.mas.newDataClasses.Production;
import rexos.mas.newDataClasses.ProductionStep;
import rexos.mas.newDataClasses.ProductionStepStatus;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.mongodb.DB;
import com.mongodb.DBObject;
//import com.mongodb.MongoClient;
import com.mongodb.Mongo;

@SuppressWarnings("serial")
public class SchedulerBehaviour extends OneShotBehaviour {

	private ProductAgent _productAgent;

	@Override
	public void action() {
		// Lets schedule ourself with the equiplet agents in our current list.
		_productAgent = (ProductAgent) myAgent;

		_productAgent.getProduct().getProduction()
				.getProductionEquipletMapping();
		try {
			Product product = this._productAgent.getProduct();
			Production production = product.getProduction();
			ArrayList<ProductionStep> psa = production.getProductionSteps();
			// debug
			// System.out.println("SIZEEEEE "+psa.length);
			System.out.println("Commence Scheduling!");
			for (ProductionStep ps : psa) {
				if (ps.getStatus() == ProductionStepStatus.STATE_TODO) {
					
					long PA_id = ps.getId();
					// System.out.println("SIZEEEEE "+production.getProductionEquipletMapping()
					// .getEquipletsForProductionStep(PA_id).keySet().size());

					Scheduler(production.getProductionEquipletMapping()
							.getEquipletsForProductionStep((int)PA_id).keySet(), ps);
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * Scheduler function schedules the given production step
	 * 
	 * @param equipletList
	 * @param productionStep
	 * @throws Exception
	 */
	public void Scheduler(Set<AID> equipletList, ProductionStep productionstep)
			throws Exception {
		Schedule[] schedules;
		// load set into arraylist
		List<AID> equipletlist = new ArrayList<AID>(equipletList);
		System.out.println("No. of equiplets for step: "
				+ productionstep.getId() + ": " + equipletlist.size());
		// Make connection with database
		Mongo mongoClient = null;
		System.out.println("Scheduler Started");

		try {
			mongoClient = new Mongo("145.89.191.131");// 145.89.191.131 is
															// hu
			// server
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}

		DB db = mongoClient.getDB("ScheduleBlackBoard");// set db to use

		// authenticating mongodb
		// boolean auth = db.authenticate("root", char['g','e','e','n']);
		// end connecting

		// extract data of every equiplet their mongoDB to Object Array
		int scheduleCount = 0;
		FreeTimeSlot[] freetimes;
		for (int i = 0; i < equipletlist.size(); i++) {
			// old name is eq1
			List<DBObject> data = db
					.getCollection(equipletlist.get(i).getName()).find()
					.toArray();// nameOfCollection should be 'schedule'
			scheduleCount += data.size();
		}

		// intialise object Schedule and object FreeTimeSlot arrays
		schedules = new Schedule[scheduleCount];
		freetimes = new FreeTimeSlot[scheduleCount];

		// get every scheduled timeslot of every equiplet
		for (int extract = 0; extract < equipletlist.size(); extract++) {
			List<DBObject> data = db
					.getCollection(equipletlist.get(extract).getName()).find()
					.toArray();// nameOfCollection should be 'schedule'
			for (int i = 0; i < data.size(); i++) {
				// debug
				System.out.println(data.get(i).toString());

				double b = (Double) data.get(i).get("startTime");
				int stati = (int) b;

				double c = (Double) data.get(i).get("duration");
				int dur = (int) c;

				// add scheduled timeslot to array of scheduled timeslots and
				// mention which equiplet
				schedules[i] = this.new Schedule(stati, dur, equipletlist.get(
						extract).getName());

				// debug
				System.out.println(schedules[i].toString());
			}
		}
		// break connection
		mongoClient.close();

		// initialise timeslot to start checking and temporarily value for
		// calculation
		int startTimeSlot = 0;
		int freetimeslotCounter = 0;

		// check within every schedule of the schedules array for free timeslots
		// and add them to the free time slots array
		for (int run = 0; run < schedules.length; run++) {
			if (schedules[run].getStartTime() > startTimeSlot) {
				if (schedules.length > (run + 1)) {
					if (schedules[run].getDeadline() < schedules[(run + 1)]
							.getStartTime()) {
						int freeTimeSlot = schedules[(run + 1)].getStartTime()
								- schedules[run].getDeadline() - 1;
						int timeslotToSchedule = (schedules[run].getDeadline() + 1);

						// debug
						System.out.println("Vrij tijd sloten: " + freeTimeSlot
								+ " startend op tijdslot: "
								+ timeslotToSchedule);

						freetimes[freetimeslotCounter] = this.new FreeTimeSlot(
								timeslotToSchedule, freeTimeSlot,
								schedules[run].getEquipletName());
					}
				}
			}
		}

		// Startslot which need to be scheduled
		FreeTimeSlot freetimeslotEq = null;
		// number of timeslots to schedule
		int timeslotsToSchedule = productionstep.getRequiredTimeSlots();

		// calculate freetime slot and asign them to the above intialized values
		if (freetimes.length > 1) {
			for (int chooseTimeSlot = 1; chooseTimeSlot < freetimes.length; chooseTimeSlot++) {
				if (freetimes[chooseTimeSlot].getDuration() <= timeslotsToSchedule) {
					if (freetimes[chooseTimeSlot].getStartTime() < freetimes[chooseTimeSlot - 1]
							.getStartTime()) {
						freetimeslotEq = freetimes[chooseTimeSlot];
					}
				}
			}
		}

		// init AID
		AID equipletAID = null;
		// get the equiplet from the timeslot
		for (int i = 0; i < equipletlist.size(); i++) {

			if (freetimeslotEq != null
					&& equipletlist.get(i).getName()
							.equals(freetimeslotEq.getEquipletName())) {
				equipletAID = equipletlist.get(i);
			}
			// debug
			System.out.println("NAME:" + equipletlist.get(i));
		}

		// debug
		// System.out.println("AID: "+equipletAID.getName());

		// send the message to the equiplet to schedule the timeslot
		ACLMessage msg = new ACLMessage(ACLMessage.INFORM);

		if (freetimeslotEq != null) {
			msg.setConversationId(_productAgent.generateCID());
			msg.setOntology("ScheduleStep");
			msg.setContent("" + freetimeslotEq.getStartTime());
			msg.addReceiver(equipletAID);
			myAgent.send(msg);

			// debug
			System.out.println("Send Timeslot to EQ");
		} else {
			// debug
			System.out.println("No Timeslot asigned. \n");
		}

	}

	private class FreeTimeSlot {
		private int startTime = -1;
		private int duration = -1;
		private String equipletName = "";

		public FreeTimeSlot(int start, int dura, String equiplet) {
			this.startTime = start;
			this.duration = dura;
			this.equipletName = equiplet;
		}

		public String getEquipletName() {
			return this.equipletName;
		}

		public int getStartTime() {
			return this.startTime;
		}

		public int getDuration() {
			return this.duration;
		}
	}

	private class Schedule {
		private int startTime = -1;
		private int duration = -1;
		private int deadline = -1;
		private String equipletName = "";

		public Schedule(int start, int dura, String equiplet) {
			this.startTime = start;
			this.duration = dura;
			this.deadline = start + dura - 1;
			this.equipletName = equiplet;
		}

		public int getStartTime() {
			return this.startTime;
		}

		@SuppressWarnings("unused")
		public void setStartTime(int newStartTime) {
			this.startTime = newStartTime;
		}

		public String getEquipletName() {
			return this.equipletName;
		}

		public int getDeadline() {
			return this.deadline;
		}

		@SuppressWarnings("unused")
		public void setDeadline(int newDeadline) {
			this.deadline = newDeadline;
		}

		public String toString() {
			return "{ startTime:" + startTime + ", duration:" + duration
					+ ", deadline:" + deadline + ", EquipletName:"
					+ equipletName + " }";
		}
	}
}
