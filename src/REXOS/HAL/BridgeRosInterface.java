package HAL;

import java.util.HashMap;
import java.util.Map;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.dataTypes.ModuleIdentifier;
import HAL.listeners.EquipletCommandListener.EquipletCommandStatus;
import HAL.listeners.ViolationListener.ViolationType;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import generic.Mast;

/**
 * A TestInterface
 * ---
 * rosBridge Library style
 * 
 * @author Lars Veenendaal
 * 
 */



public class BridgeRosInterface extends RosInterface {
	private Map<Integer, HardwareStep> hardwareSteps;
	private Integer hardwareStepId;
	
	Ros rosNode;
	Topic hardwareStepStatusTopic;
	Topic equipletCommandStatusTopic;
	Topic stateChangedTopic;
	Topic modeChangedTopic;
	Topic hardwareStepsTopic;
	Topic equipletCommandsTopic;
	Topic violationsTopic;
	
	AbstractHardwareAbstractionLayer hal;
	
	protected BridgeRosInterface(final AbstractHardwareAbstractionLayer hal) {
		this.hal = hal;
		this.hardwareSteps = new HashMap<Integer, HardwareStep>();
		this.hardwareStepId = 1;
		this.rosNode = new Ros();
		
		rosNode.connect();
		
		String path;
		if(hal.isShadow() == true) {
			 path = "shadow/" + hal.getEquipletName() + "/";
		} else {
			 path = hal.getEquipletName() + "/";
		}
		
		hardwareStepStatusTopic = new Topic(rosNode, path + "hardwareStepStatus", "std_msgs/String");
		hardwareStepStatusTopic.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				try {
					JSONObject messageJson = new JSONObject(message.toJsonObject().getString("data"));
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
					BridgeRosInterface.this.onHardwareStepStatusChanged(step);
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading hardware step status changed failed", ex);
				}
			}
		});
		equipletCommandStatusTopic = new Topic(rosNode, path + "equipletCommandStatus", "std_msgs/String");
		equipletCommandStatusTopic.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				try {
					JSONObject messageJson = new JSONObject(message.toJsonObject().getString("data"));
					String status = messageJson.getString("status");
					BridgeRosInterface.this.onEquipletCommandStatusChanged(EquipletCommandStatus.valueOf(status));
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading equiplet command status changed failed", ex);
				}
			}
		});
		stateChangedTopic = new Topic(rosNode, path + "stateChanged", "std_msgs/String");
		stateChangedTopic.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				try {
					JSONObject messageJson = new JSONObject(message.toJsonObject().getString("data"));
					String state = messageJson.getString("state");
					if(messageJson.has("moduleIdentifier")) {
						JSONObject moduleIdentifierJson = messageJson.getJSONObject("moduleIdentifier");
						ModuleIdentifier identifier = new ModuleIdentifier(moduleIdentifierJson.getString("manufacturer"), 
								moduleIdentifierJson.getString("typeNumber"), moduleIdentifierJson.getString("serialNumber"));
						Module module = hal.getModuleFactory().getItemForIdentifier(identifier);
						BridgeRosInterface.this.onModuleStateChanged(module, Mast.State.valueOf(state));
					} else {
						BridgeRosInterface.this.onEquipletStateChanged(Mast.State.valueOf(state));
					}
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading state changed failed", ex);
				}
			}
		});
		modeChangedTopic = new Topic(rosNode, path + "modeChanged", "std_msgs/String");
		modeChangedTopic.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				try {
					JSONObject messageJson = new JSONObject(message.toJsonObject().getString("data"));
					String mode = messageJson.getString("mode");
					if(messageJson.has("moduleIdentifier")) {
						JSONObject moduleIdentifierJson = messageJson.getJSONObject("moduleIdentifier");
						ModuleIdentifier identifier = new ModuleIdentifier(moduleIdentifierJson.getString("manufacturer"), 
								moduleIdentifierJson.getString("typeNumber"), moduleIdentifierJson.getString("serialNumber"));
						Module module = hal.getModuleFactory().getItemForIdentifier(identifier);
						BridgeRosInterface.this.onModuleModeChanged(module, Mast.Mode.valueOf(mode));
					} else {
						BridgeRosInterface.this.onEquipletModeChanged(Mast.Mode.valueOf(mode));
					}
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading mode changed failed", ex);
				}
			}
		});
		violationsTopic = new Topic(rosNode, path + "violationOccured", "std_msgs/String");
		violationsTopic.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				try {
					JSONObject messageJson = new JSONObject(message.toJsonObject().getString("data"));
					ViolationType violationType = ViolationType.valueOf(messageJson.getString("type"));
					String violationMessage = messageJson.getString("message");
					BridgeRosInterface.this.onViolationOccured(violationType, violationMessage);
				} catch (JSONException ex) {
					Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.ERROR, "Reading mode changed failed", ex);
				}
			}
		});
		
		hardwareStepsTopic = new Topic(rosNode, path + "hardwareSteps", "std_msgs/String");
		hardwareStepsTopic.advertise();
		equipletCommandsTopic = new Topic(rosNode, path + "equipletCommands", "std_msgs/String");
		equipletCommandsTopic.advertise();
	}

	@Override
	public void postHardwareStep(HardwareStep hardwareStep) {
		try{
			JSONObject hardwareStepJson = hardwareStep.toJSON();
			hardwareStepJson.put("id", String.valueOf(hardwareStepId));
			JSONObject messageJson = new JSONObject();
			messageJson.put("data", hardwareStepJson.toString());
			Message message = new Message(messageJson.toString());
			
			synchronized (hardwareSteps) {
				hardwareStepsTopic.publish(message);
				hardwareSteps.put(hardwareStepId, hardwareStep);
				hardwareStepId++;
			}
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
		}
	}

	@Override
	public void postEquipletCommand(JSONObject equipletCommand) {
		try{
			JSONObject messageJson = new JSONObject();
			messageJson.put("data", equipletCommand.toString());
			Message message = new Message(messageJson.toString());
			equipletCommandsTopic.publish(message);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_ROS_INTERFACE, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
		}
	}

	@Override
	public void shutdown() {
	    rosNode.disconnect();
	}
}
