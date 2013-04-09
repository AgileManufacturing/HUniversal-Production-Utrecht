package scheduling;

//jade imports
import jade.core.AID;
import jade.core.Agent;

//mongodb imports
import com.mongodb.MongoClient;
import com.mongodb.MongoException;
import com.mongodb.WriteConcern;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.DBCursor;
import com.mongodb.ServerAddress;

//Required imports
import jade.domain.AMSService;
import jade.domain.FIPAAgentManagement.*;
import jade.lang.acl.ACLMessage;

//usual imports
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import productAgent.ProductAgent;


public class Scheduler {
	
	public Scheduler(AID[] equipletList, Object productionStep)throws Exception{
		Schedule[] schedules;
		
		//MongoClient mongoClient = new MongoClient();
		// or
		
		//Make connection with database
		MongoClient mongoClient =null;
		try {
			mongoClient = new MongoClient( "localhost" );//145.89.191.131 is hu server
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		
		// or
		//MongoClient mongoClient = new MongoClient( "localhost" , 27017 );
		// or, to connect to a replica set, supply a seed list of members
		/*MongoClient mongoClient = new MongoClient(Arrays.asList(new ServerAddress("localhost", 27017),
		                                      new ServerAddress("localhost", 27018),
		                                      new ServerAddress("localhost", 27019)));*/
		
		DB db = mongoClient.getDB( "ScheduleBlackBoard" );//set db to use
		
		//authenticating mongodb
		//boolean auth = db.authenticate("root", char['g','e','e','n']);
		//end connecting
		
		//extract data of every equiplet their mongoDB to Object Array
		int scheduleCount = 0;
		FreeTimeSlot[] freetimes;
		for(int i = 0; i<equipletList.length;i++){
			//old name is eq1
			List<DBObject> data = db.getCollection(equipletList[i].getName()).find().toArray();//nameOfCollection should be 'schedule'
			scheduleCount += data.size();
		}
		
		//intialise object Schedule and object FreeTimeSlot arrays
		schedules= new Schedule[scheduleCount];
		freetimes = new FreeTimeSlot[scheduleCount];
		
		//get every scheduled timeslot of every equiplet
		for(int extract = 0; extract<equipletList.length;extract++){
			List<DBObject> data = db.getCollection(equipletList[extract].getName()).find().toArray();//nameOfCollection should be 'schedule'
			for(int i = 0; i < data.size(); i++){
				//debug
				System.out.println(data.get(i).toString());
				
				double b = (Double) data.get(i).get("startTime");
				int stati = (int)b;
				
				double c = (Double) data.get(i).get("duration");
				int dur = (int)c;
				
				//add scheduled timeslot to array of scheduled timeslots and mention which equiplet
				schedules[i] = this.new Schedule(stati, dur, equipletList[extract].getName());
				
				//debug
				System.out.println(schedules[i].toString());
			}
		}
		//break connection
		mongoClient.close();
		
		//initialise timeslot to start checking and temporarily value for calculation
		int startTimeSlot = 0;
		int freetimeslotCounter = 0;
		
		//check within every schedule of the schedules array for free timeslots and add them to the free time slots array 
		for(int run = 0; run < schedules.length;run++){
			if(schedules[run].getStartTime() > startTimeSlot){
				if(schedules.length > (run+1)){
					if(schedules[run].getDeadline() < schedules[(run+1)].getStartTime()){
						int freeTimeSlot = schedules[(run+1)].getStartTime() - schedules[run].getDeadline()-1;
						int timeslotToSchedule= (schedules[run].getDeadline()+1);
						
						
						//debug
						System.out.println("Vrij tijd sloten: "+freeTimeSlot + " startend op tijdslot: "+ timeslotToSchedule );
						
						freetimes[freetimeslotCounter] = this.new FreeTimeSlot(timeslotToSchedule, freeTimeSlot, schedules[run].getEquipletName());
					}
				}
			}
		}
		
		//set startslot which need to be scheduled
		int startSlot = 0;
		FreeTimeSlot freetimeslotEq = null;
		
		//calculate freetime slot and asign them to the above intialized values
		if(freetimes.length > 1){
			for(int chooseTimeSlot = 1;chooseTimeSlot < freetimes.length; chooseTimeSlot++){
				if(freetimes[chooseTimeSlot].getStartTime() < freetimes[chooseTimeSlot-1].getStartTime()){
					startSlot = freetimes[chooseTimeSlot].getStartTime();
					freetimeslotEq = freetimes[chooseTimeSlot];
				}
			}
		}
		
		//init AID
		AID equipletAID =null;
		//get the equiplet from the timeslot
		for (int i=0; i<equipletList.length;i++)
        {
            if(equipletList[i].getName().equals(freetimeslotEq.getEquipletName())){
            	equipletAID = equipletList[i];
            }
        }
		
		//send the message to the equiplet to schedule the timeslot
		ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
        msg.setContent( "Schedule" );

        msg.addReceiver(equipletAID); 

        //send(msg);
		
		//equipletAgent.scheduleProductionStep(startSlot);
		
	}
	
	private class FreeTimeSlot{
		private int startTime = -1;
		private int duration = -1;
		private String equipletName = "";
		
		public FreeTimeSlot(int start, int dura, String equiplet){
			this.startTime = start;
			this.duration = dura;
			this.equipletName = equiplet;
		}
		
		public String getEquipletName(){
			return this.equipletName;
		}
		
		public int getStartTime(){
			return this.startTime;
		}
		public int getDuration(){
			return this.duration;
		}
	}
	
	private class Schedule{
		private int startTime = -1;
		private int duration = -1;
		private int deadline = -1;
		private String equipletName = "";
		
		public Schedule(int start, int dura, String equiplet){
			this.startTime = start;
			this.duration = dura;
			this.deadline = start+dura-1;
			this.equipletName = equiplet;
		}
		
		public int getStartTime(){
			return this.startTime;
		}
		public void setStartTime(int newStartTime){
			this.startTime = newStartTime;
		}
		
		public String getEquipletName(){
			return this.equipletName;
		}
		
		public int getDeadline(){
			return this.deadline;
		}
		public void setDeadline(int newDeadline){
			this.deadline = newDeadline;
		}
		
		public String toString(){
			return "{ startTime:"+startTime+", duration:"+duration+", deadline:"+deadline+", EquipletName:"+equipletName + " }";
		}
	}
}
