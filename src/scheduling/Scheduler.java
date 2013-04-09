package scheduling;

//jade imports
import jade.core.AID;

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

//usual imports
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.List;
import java.util.Set;


public class Scheduler {
	
	public Scheduler(AID[] equipletList, Object productionStep)throws Exception{
		Schedule[] schedules;
		
		//MongoClient mongoClient = new MongoClient();
		// or
		
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
		
		//extract data of mongoDB to Object Array
		int scheduleCount = 0;
		for(int i = 0; i<equipletList.length;i++){
			//old name is eq1
			List<DBObject> data = db.getCollection(equipletList[i].getName()).find().toArray();//nameOfCollection should be 'schedule'
			scheduleCount += data.size();
		}
		schedules= new Schedule[scheduleCount];
		
		
		FreeTimeSlot[] freetimes = new FreeTimeSlot[data.size()];
		
		for(int i = 0; i < data.size(); i++){
			System.out.println(data.get(i).toString());
			
			double b = (Double) data.get(i).get("startTime");
			int stati = (int)b;
			
			double c = (Double) data.get(i).get("duration");
			int dur = (int)c;
			
			schedules[i] = this.new Schedule(stati, dur);
			
			System.out.println("Schedule "+i+" added");
			System.out.println(schedules[i].toString());
		}
		mongoClient.close();
		
		int startTimeSlot = 0;
		int freetimeslotCounter = 0;
		
		for(int run = 0; run < schedules.length;run++){
			if(schedules[run].getStartTime() > startTimeSlot){
				if(schedules.length > (run+1)){
					if(schedules[run].getDeadline() < schedules[(run+1)].getStartTime()){
						int freeTimeSlot = schedules[(run+1)].getStartTime() - schedules[run].getDeadline()-1;
						int timeslotToSchedule= (schedules[run].getDeadline()+1);
						System.out.println("Vrij tijd sloten: "+freeTimeSlot + " startend op tijdslot: "+ timeslotToSchedule );
						freetimes[freetimeslotCounter] = this.new FreeTimeSlot(timeslotToSchedule, freeTimeSlot);
					}
				}
			}
		}
		
		int startSlot = 0;
		if(freetimes.length > 1){
			for(int chooseTimeSlot = 1;chooseTimeSlot < freetimes.length; chooseTimeSlot++){
				if(freetimes[chooseTimeSlot].getStartTime() < freetimes[chooseTimeSlot-1].getStartTime()){
					startSlot = freetimes[chooseTimeSlot].getStartTime();
				}
			}
		}
		//equipletAgent.scheduleProductionStep(startSlot);
		
	}
	
	private class FreeTimeSlot{
		private int startTime = -1;
		private int duration = -1;
		
		public FreeTimeSlot(int start, int dura){
			this.startTime = start;
			this.duration = dura;
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
		
		public Schedule(int start, int dura){
			this.startTime = start;
			this.duration = dura;
			this.deadline = start+dura-1;
		}
		
		public int getStartTime(){
			return this.startTime;
		}
		public void setStartTime(int newStartTime){
			this.startTime = newStartTime;
		}
		
		public int getDuration(){
			return this.duration;
		}
		public void setDuration(int newDuration){
			this.duration = newDuration;
		}
		
		public int getDeadline(){
			return this.deadline;
		}
		public void setDeadline(int newDeadline){
			this.deadline = newDeadline;
		}
		
		public String toString(){
			return "{ startTime:"+startTime+", duration:"+duration+", deadline:"+deadline+" }";
		}
	}
}
