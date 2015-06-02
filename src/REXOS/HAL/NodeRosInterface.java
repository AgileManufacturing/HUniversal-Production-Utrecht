package HAL;

import generic.Mast;

import java.util.HashMap;
import java.util.Map;

import org.json.JSONException;
import org.json.JSONObject;
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

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.dataTypes.ModuleIdentifier;
import HAL.listeners.EquipletCommandListener.EquipletCommandStatus;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

public class NodeRosInterface extends RosInterface implements NodeMain {
	private Subscriber<std_msgs.String> hardwareStepStatusChangedSubscriber;
	private Subscriber<std_msgs.String> equipletCommandStatusChangedSubscriber;
	private Subscriber<std_msgs.String> stateChangedSubscriber;
	private Subscriber<std_msgs.String> modeChangedSubscriber;
	
	private Publisher<std_msgs.String> hardwareStepPublisher;
	private Publisher<std_msgs.String> equipletCommandPublisher;
	
	private Map<Integer, HardwareStep> hardwareSteps;
	private Integer hardwareStepId;
	
	private NodeMainExecutor executor;
	
	private boolean hasNodeStarted;

	AbstractHardwareAbstractionLayer hal;

	public NodeRosInterface(AbstractHardwareAbstractionLayer hal) {
		this.hal = hal;
		this.hardwareSteps = new HashMap<Integer, HardwareStep>();
		this.hardwareStepId = 1;
		this.hasNodeStarted = false;
		
		//ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(2);
		//executor = DefaultNodeMainExecutor.newDefault(scheduler);
		executor = DefaultNodeMainExecutor.newDefault();
		executor.execute(this, NodeConfiguration.newPrivate());
		
		synchronized (this) {
			try {
				if(hasNodeStarted == false) {
					System.out.println("Waiting for node");
					wait();
				}
				// TODO wait for the topics to come online, nicer solution required
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.out.println("Done");
		}
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		if(hal.isShadow() == true) {
			return GraphName.of("shadow/" + hal.getEquipletName() + "_HAL/");
		} else {
			return GraphName.of(hal.getEquipletName() + "_HAL/");
		}
	}

	@Override
	public void onStart(ConnectedNode node) {
		String path = hal.getEquipletName() + "/";
		
		hardwareStepStatusChangedSubscriber = node.newSubscriber(path + "hardwareStepStatus", std_msgs.String._TYPE);
		hardwareStepStatusChangedSubscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				try {
					JSONObject messageJson = new JSONObject(message.getData());
					Integer id = Integer.valueOf(messageJson.getString("id"));
					String status = messageJson.getString("status");
					
					HardwareStep step;
					synchronized (hardwareSteps) {
						step = hardwareSteps.get(id);
						step.setStatus(HardwareStepStatus.valueOf(status));
						if(step.getStatus() == HardwareStepStatus.DONE || step.getStatus() == HardwareStepStatus.FAILED) {
							// we are done with this step
							hardwareSteps.remove(id);
						}
					}
					NodeRosInterface.this.onHardwareStepStatusChanged(step);
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading hardware step status changed failed", ex);
				}
			}
		}, 10);
		equipletCommandStatusChangedSubscriber = node.newSubscriber(path + "equipletCommandStatus", std_msgs.String._TYPE);
		equipletCommandStatusChangedSubscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				try {
					JSONObject messageJson = new JSONObject(message.getData());
					String status = messageJson.getString("status");
					NodeRosInterface.this.onEquipletCommandStatusChanged(EquipletCommandStatus.valueOf(status));
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading equiplet command status changed failed", ex);
				}
			}
		}, 10);
		stateChangedSubscriber = node.newSubscriber(path + "stateChanged", std_msgs.String._TYPE);
		stateChangedSubscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				try {
					JSONObject messageJson = new JSONObject(message.getData());
					String state = messageJson.getString("state");
					if(messageJson.has("moduleIdentifier")) {
						JSONObject moduleIdentifierJson = messageJson.getJSONObject("moduleIdentifier");
						ModuleIdentifier identifier = new ModuleIdentifier(moduleIdentifierJson.getString("manufacturer"), 
								moduleIdentifierJson.getString("typeNumber"), moduleIdentifierJson.getString("serialNumber"));
						Module module = hal.getModuleFactory().getItemForIdentifier(identifier);
						NodeRosInterface.this.onModuleStateChanged(module, Mast.State.valueOf(state));
					} else {
						NodeRosInterface.this.onEquipletStateChanged(Mast.State.valueOf(state));
					}
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading state changed failed", ex);
				}
			}
		}, 10);
		modeChangedSubscriber = node.newSubscriber(path + "modeChanged", std_msgs.String._TYPE);
		modeChangedSubscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				try {
					JSONObject messageJson = new JSONObject(message.getData());
					String mode = messageJson.getString("mode");
					if(messageJson.has("moduleIdentifier")) {
						JSONObject moduleIdentifierJson = messageJson.getJSONObject("moduleIdentifier");
						ModuleIdentifier identifier = new ModuleIdentifier(moduleIdentifierJson.getString("manufacturer"), 
								moduleIdentifierJson.getString("typeNumber"), moduleIdentifierJson.getString("serialNumber"));
						Module module = hal.getModuleFactory().getItemForIdentifier(identifier);
						NodeRosInterface.this.onModuleModeChanged(module, Mast.Mode.valueOf(mode));
					} else {
						NodeRosInterface.this.onEquipletModeChanged(Mast.Mode.valueOf(mode));
					}
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading mode changed failed", ex);
				}
			}
		}, 10);
		
		
		hardwareStepPublisher = node.newPublisher(path + "hardwareSteps", std_msgs.String._TYPE);
		equipletCommandPublisher = node.newPublisher(path + "equipletCommands", std_msgs.String._TYPE);
		
		synchronized (this) {
			hasNodeStarted = true;
			this.notifyAll();
		}
	}
	
	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onShutdownComplete(Node node) {
		Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "interface has been shut down");
	}

	@Override
	public void onError(Node node, Throwable throwable) {
		Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.EMERGENCY, "Error occured in rosjava", throwable);
	}

	@Override
	public synchronized void postHardwareStep(HardwareStep hardwareStep) {
		try {
			std_msgs.String message = hardwareStepPublisher.newMessage();
			JSONObject hardwareStepJson = hardwareStep.toJSON();
			hardwareStepJson.put("id", String.valueOf(hardwareStepId));
			message.setData(hardwareStepJson.toString());
			
			synchronized (hardwareSteps) {
				hardwareStepPublisher.publish(message);
				hardwareSteps.put(hardwareStepId, hardwareStep);
				hardwareStepId++;
			}
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
		}
	}

	@Override
	public synchronized void postEquipletCommand(JSONObject equipletCommand) {
		std_msgs.String message = equipletCommandPublisher.newMessage();
		message.setData(equipletCommand.toString());
		equipletCommandPublisher.publish(message);
	}

	@Override
	public void shutdown() {
		Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.DEBUG, "Shutting down interface");
		executor.shutdown();
		executor.getScheduledExecutorService().shutdownNow();
	}
}