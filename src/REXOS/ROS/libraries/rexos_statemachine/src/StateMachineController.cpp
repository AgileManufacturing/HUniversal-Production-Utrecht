#include <rexos_statemachine/StateMachineController.h>
#include <rexos_logger/rexos_logger.h>

namespace rexos_statemachine{
	StateMachineController::StateMachineController(std::string advertisementPath) :
			changeStateActionClient	(nodeHandle, advertisementPath + "change_state"),
			changeModeActionClient	(nodeHandle, advertisementPath + "change_mode"),
			currentState(STATE_OFFLINE),
			currentMode(MODE_SERVICE)
		{
		stateUpdateServiceServer = nodeHandle.advertiseService	(advertisementPath + "state_update",
				&StateMachineController::onStateChangeServiceCallback, this);
		modeUpdateServiceServer = nodeHandle.advertiseService		(advertisementPath + "mode_update",
				&StateMachineController::onModeChangeServiceCallback, this);
		
		REXOS_INFO_STREAM("Setting state action client: " 	<< advertisementPath << "change_state");
		REXOS_INFO_STREAM("Setting mode action client: " 		<< advertisementPath << "change_mode");
		REXOS_INFO_STREAM("Setting state update server: " 	<< advertisementPath << "state_update");
		REXOS_INFO_STREAM("Setting mode update server: " 		<< advertisementPath << "mode_update");
	}
	
	rexos_statemachine::State StateMachineController::getCurrentState(){
		return currentState;
	}

	rexos_statemachine::Mode StateMachineController::getCurrentMode(){
		return currentMode;
	}
	
	void StateMachineController::changeState(rexos_statemachine::State state) {
		rexos_statemachine::ChangeStateGoal goal;
		goal.desiredState = state;
		if(changeStateActionClient.waitForServer(ros::Duration(30.0)) == false) {
			throw std::runtime_error("changeStateActionServer took too long to respond");
		}
		changeStateActionClient.sendGoal(goal);
	}

	void StateMachineController::changeMode(rexos_statemachine::Mode mode) {
		rexos_statemachine::ChangeModeGoal goal;
		goal.desiredMode = mode;
		if(changeModeActionClient.waitForServer(ros::Duration(30.0)) == false) {
			throw std::runtime_error("changeModeActionServer took too long to respond");
		}
		changeModeActionClient.sendGoal(goal);
	}
	
	bool StateMachineController::onStateChangeServiceCallback(StateUpdateRequest &req, StateUpdateResponse &res){
		rexos_statemachine::State previousState = currentState;
		currentState = static_cast<rexos_statemachine::State>(req.state);

		onStateChange(currentState, previousState);
		return true;
	}
	
	bool StateMachineController::onModeChangeServiceCallback(ModeUpdateRequest &req, ModeUpdateResponse &res){
		rexos_statemachine::Mode previousMode = currentMode;
		currentMode = static_cast<rexos_statemachine::Mode>(req.mode);

		onModeChange(currentMode, previousMode);
		return true;
	}
}
