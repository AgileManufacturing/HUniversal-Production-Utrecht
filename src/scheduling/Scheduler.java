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
import java.util.Arrays;
import java.util.List;
import java.util.Set;


public class Scheduler {
	
	public Scheduler(AID[] equipletList, Object productionStep)throws Exception{
		//blackboard.connect(scheduleBlackboard);
		//MongoClient mongoClient = new MongoClient();
		// or
		MongoClient mongoClient = new MongoClient( "localhost" );//145.89.191.131 is hu server
		// or
		//MongoClient mongoClient = new MongoClient( "localhost" , 27017 );
		// or, to connect to a replica set, supply a seed list of members
		/*MongoClient mongoClient = new MongoClient(Arrays.asList(new ServerAddress("localhost", 27017),
		                                      new ServerAddress("localhost", 27018),
		                                      new ServerAddress("localhost", 27019)));*/
		
		DB db = mongoClient.getDB( "scheduleBlackboard" );//set db to use
		
		//authenticating mongodb
		//boolean auth = db.authenticate("root", char['g','e','e','n']);
		//end connecting
		
		//get current collection names(table names)
		Set<String> colls = db.getCollectionNames();
		//show current collection data
		//for (String s : colls) {
			List<DBObject> data = db.getCollection("eq1").find().toArray();//nameOfCollection should be 'schedule'
			for(int i = 0; i < data.size(); i++){
				System.out.println(data.get(i).toString());
				
			}
		//}
		
		//TODO tijdsloten berekenen welke vrij zijn	en eerste vrije slot kiezen
		//( bepaald ook welke equiplet het wordt indien meerdere equiplets beschikbaar)	
		int startTimeSlot = 1;
		
		schedule(equipletList[0], startTimeSlot);
		
	}
	
	private void schedule(AID equiplet, int startTimeslot){
		//blackboard.place(equipletID, startTimeSlot);
	}
}
