package productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import libraries.blackboardJavaClient.src.nl.hu.client.BlackboardClient;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionStep;

import com.mongodb.DBObject;

@SuppressWarnings("serial")
public class SchedulerBehaviour extends OneShotBehaviour {

	private ProductAgent _productAgent;
	private int timeslotsToSchedule = 0;
	
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
			
			
			//debug
			System.out.println("NUMBER OF EQUIPLETS: "+psa.size());
			
			for (ProductionStep ps : psa) {
				int PA_id = ps.getId();
				
				if(production.getProductionEquipletMapping().getEquipletsForProductionStep(PA_id).keySet().size() >0){
					this.timeslotsToSchedule =  production.getProductionEquipletMapping().getTimeSlotsForEquiplet(PA_id, (AID)production.getProductionEquipletMapping()
							.getEquipletsForProductionStep(PA_id).keySet().toArray()[0]).intValue();
				}
				
				System.out.println("-------------------");
				System.out.println("STEP_ID:"+PA_id+" NUMBER OF EQ AVAILABLE: "+production.getProductionEquipletMapping()
						.getEquipletsForProductionStep(PA_id).keySet().size());
				System.out.println("STEP_ID:"+ps.getId()+" requires "+this.timeslotsToSchedule+" timeslots");
				System.out.println("-------------------");
				
				Scheduler(production.getProductionEquipletMapping()
						.getEquipletsForProductionStep(PA_id).keySet(), ps);
				
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
		//load set into arraylist
		 List<AID> equipletlist = new ArrayList<AID>(equipletList);
	
		//Make connection with blackboard
		BlackboardClient bbc = new BlackboardClient("145.89.191.131");
		bbc.setDatabase("ScheduleBlackBoard");
		
		//debug
		System.out.println("Scheduler Started");

		// authenticating mongodb
		// boolean auth = db.authenticate("root", char['g','e','e','n']);
		// end connecting

		// extract data of every equiplet their mongoDB to Object Array
		int scheduleCount = 0;
		FreeTimeSlot[] freetimes;
		for (int i = 0; i < equipletlist.size(); i++) {
			// old name is eqa1
			bbc.setCollection(equipletlist.get(i).getLocalName().toString());
			List<DBObject> blackBoard = bbc.findDocuments(" ");
			//List<DBObject> data = db.getCollection(equipletlist.get(i).getLocalName()).find().toArray();// nameOfCollection should be 'schedule'
			scheduleCount += blackBoard.size();
			
			//debug
			System.out.println("----- Get list of the already scheduled data -------");
			System.out.println(""+equipletlist.get(i).getLocalName());
			System.out.println();
		}
		
		//debug
		System.out.println("--------- ");
		System.out.println("ScheduleCount: "+scheduleCount);
		System.out.println();

		// intialise object Schedule and object FreeTimeSlot arrays
		schedules = new Schedule[scheduleCount];
		freetimes = new FreeTimeSlot[scheduleCount];

		// get every scheduled timeslot of every equiplet
		for (int extract = 0; extract < equipletlist.size(); extract++) {
			bbc.setCollection(equipletlist.get(extract).getLocalName().toString());
			List<DBObject> blackBoard = bbc.findDocuments(" ");
			
			//List<DBObject> data = db.getCollection(equipletlist.get(extract).getLocalName()).find().toArray();// nameOfCollection should be 'schedule'
			for (int i = 0; i < blackBoard.size(); i++) {

				double b = (Double) blackBoard.get(i).get("startTime");
				int stati = (int) b;

				double c = (Double) blackBoard.get(i).get("duration");
				int dur = (int) c;

				// add scheduled timeslot to array of scheduled timeslots and
				// mention which equiplet
				schedules[i] = this.new Schedule(stati, dur, equipletlist.get(
						extract).getName());
			}
		}

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
						System.out.println(freetimeslotCounter+" : "+freetimes[freetimeslotCounter].toString());
						System.out.println();
						freetimeslotCounter++;
					}
				}
			}
		}

		// Startslot which need to be scheduled
		FreeTimeSlot freetimeslotEq = null;
		System.out.println("---- Number of timeslots to schedule -----");
		System.out.println("Timeslots to schedule: "+timeslotsToSchedule);
		System.out.println();
		
		// calculate freetime slot and asign them to the above intialized values
		if (freetimes.length > 1) {
			System.out.println("Free time Slots:"+freetimes.length);
			
			for (int chooseTimeSlot = 0; chooseTimeSlot < freetimes.length; chooseTimeSlot++) {
				
				if(freetimes[chooseTimeSlot] != null){
			
					if (freetimes[chooseTimeSlot].getDuration() <= timeslotsToSchedule) {
							freetimeslotEq = freetimes[chooseTimeSlot];
					}
				}
			}
		}

		// init AID
		AID equipletAID = null;
		// get the equiplet from the timeslot
		for (int i = 0; i < equipletlist.size(); i++) {

			if (freetimeslotEq != null && equipletlist.get(i).getName()
					.equals(freetimeslotEq.getEquipletName())) {
				equipletAID = equipletlist.get(i);
			}
		}
		//debug
		System.out.println("------- Equiplet which gains Free time slot --------");
		System.out.println("AID NAME:"+equipletAID+ "");
		System.out.println();
		
		// send the message to the equiplet to schedule the timeslot
		ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
		
		if(freetimeslotEq != null && equipletAID != null){
			msg.setConversationId(_productAgent.generateCID());
			msg.setOntology("ScheduleStep");
	        msg.setContent( ""+freetimeslotEq.getStartTime() );
	        msg.addReceiver(equipletAID);
	        myAgent.send(msg);
	        
	        //debug
	        System.out.println("Send Timeslot "+equipletAID.getName()+" to EQ");
		}else{
			//debug
			System.out.println("No Timeslot asigned.");
		}
		System.out.println();
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
		
		public String toString(){
			return "{Start TimeSlot: "+this.startTime + ", Duration: "+this.duration + ", EquipletName: "+this.equipletName+"}";
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
