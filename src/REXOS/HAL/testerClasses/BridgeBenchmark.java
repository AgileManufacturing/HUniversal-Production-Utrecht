package HAL.testerClasses;

import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;

public class BridgeBenchmark {
	public static void main(String[] args) throws InterruptedException {
		Ros rosNode = new Ros();
		rosNode.connect();
		
		final Topic bTopic = new Topic(rosNode, "roscpp_benchmark_b", "std_msgs/Int32");
		bTopic.advertise();
		
		Topic aTopic = new Topic(rosNode, "roscpp_benchmark_a", "std_msgs/Int32");
		aTopic.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				bTopic.publish(message);
			}
		});
		
		synchronized (rosNode) {
			rosNode.wait();
		}
	}
}
