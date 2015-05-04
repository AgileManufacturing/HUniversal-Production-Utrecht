package HAL;

import java.util.HashMap;
import java.util.Map;

import org.bson.types.ObjectId;
import org.json.JSONObject;

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.*;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;
import HAL.steps.HardwareStep;

/**
 * A TestInterface
 * ---
 * rosBridge Library style
 * 
 * @author Lars Veenendaal
 * 
 */



public class TestInterface extends RosInterface {

	private Map<ObjectId, HardwareStep> hardwareSteps;
	
	protected TestInterface(HardwareAbstractionLayer hal) {
		super(hal);
		// TODO Auto-generated constructor stub
		this.hardwareSteps = new HashMap<ObjectId, HardwareStep>();
		String equipletName = hal.getEquipletName();
	}

	@Override
	public void postHardwareStep(HardwareStep hardwareStep) {
		Ros ros = new Ros("localhost");
	    ros.connect();

	    Topic echo = new Topic(ros, "/echo", "std_msgs/String");
	    Message toSend = new Message("{\"data\": \"hello, world!\"}");
	    echo.publish(toSend);

	    Topic echoBack = new Topic(ros, "/echo_back", "std_msgs/String");
	    echoBack.subscribe(new TopicCallback() {
	        @Override
	        public void handleMessage(Message message) {
	            System.out.println("From ROS: " + message.toString());
	        }
	    });

	    Service addTwoInts = new Service(ros, "/add_two_ints", "rospy_tutorials/AddTwoInts");

	    ServiceRequest request = new ServiceRequest("{\"a\": 10, \"b\": 20}");
	    ServiceResponse response = addTwoInts.callServiceAndWait(request);
	    System.out.println(response.toString());

	    ros.disconnect();
	}

	@Override
	public void postEquipletCommand(JSONObject equipletCommand) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void shutdown() {
		// TODO Auto-generated method stub
		
	}


	
}
