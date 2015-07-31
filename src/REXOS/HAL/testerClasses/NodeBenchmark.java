package HAL.testerClasses;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class NodeBenchmark implements NodeMain {
	private Subscriber<std_msgs.Int32> aTopic;
	private Publisher<std_msgs.Int32> bTopic;
	
	private NodeMainExecutor executor;

	public NodeBenchmark() {
		executor = DefaultNodeMainExecutor.newDefault();
		executor.execute(this, NodeConfiguration.newPrivate());
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("node_b");
	}

	@Override
	public void onStart(ConnectedNode node) {
		bTopic = node.newPublisher("roscpp_benchmark_b", std_msgs.Int32._TYPE);
		aTopic = node.newSubscriber("roscpp_benchmark_a", std_msgs.Int32._TYPE);
		aTopic.addMessageListener(new MessageListener<std_msgs.Int32>() {
			@Override
			public void onNewMessage(std_msgs.Int32 message) {
				bTopic.publish(message);
			}
		}, 10);
	}

	@Override
	public void onError(Node arg0, Throwable arg1) {
	}

	@Override
	public void onShutdown(Node arg0) {
	}

	@Override
	public void onShutdownComplete(Node arg0) {
	}
	
	public static void main(String[] args) throws InterruptedException {
		NodeBenchmark nodeBenchmark = new NodeBenchmark();
		synchronized (nodeBenchmark) {
			nodeBenchmark.wait();
		}
	}
}