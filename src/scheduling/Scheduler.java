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
import java.util.Set;

//import blackboard;

public class Scheduler {
	
	public Scheduler(AID[] equipletList, Object productionStep)throws Exception{
		//blackboard.connect(scheduleBlackboard);
		//MongoClient mongoClient = new MongoClient();
		// or
		MongoClient mongoClient = new MongoClient( "localhost" );
		// or
		//MongoClient mongoClient = new MongoClient( "localhost" , 27017 );
		// or, to connect to a replica set, supply a seed list of members
		/*MongoClient mongoClient = new MongoClient(Arrays.asList(new ServerAddress("localhost", 27017),
		                                      new ServerAddress("localhost", 27018),
		                                      new ServerAddress("localhost", 27019)));*/
		
		DB db = mongoClient.getDB( "scheduleBlackboard" );
		//boolean auth = db.authenticate("root", char['g','e','e','n']);
		Set<String> colls = db.getCollectionNames();
		//show current collections
		for (String s : colls) {
		    System.out.println(s);
		}
		
		//end connecting
		
		//TODO tijdsloten berekenen welke vrij zijn	en eerste vrije slot kiezen
		//( bepaald ook welke equiplet het wordt indien meerdere equiplets beschikbaar)	
		int startTimeSlot = 0;
		
		schedule(equipletList[0], startTimeSlot);
		
	}
	
	private void schedule(AID equiplet, int startTimeslot){
		//blackboard.place(equipletID, startTimeSlot);
	}
}
