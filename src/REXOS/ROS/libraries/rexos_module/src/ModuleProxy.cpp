#include <rexos_module/ModuleProxy.h>

namespace rexos_module {
	ModuleProxy::ModuleProxy(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, ModuleProxyListener* moduleProxyListener) :
			ModuleInterface(equipletName, identifier, moduleProxyListener), 
			rexos_statemachine::StateMachineController(advertisementPath), 
			moduleProxyListener(moduleProxyListener),
			transitionActionServer(AbstractModule::nodeHandle, advertisementPath + "transition", 
				boost::bind(&ModuleProxy::onModuleTransitionGoalCallback, this, _1), true),
			bond(NULL)
	{
		
	}
	ModuleProxy::~ModuleProxy() {
		if(bond != NULL) {
			bond->breakBond();
			bond->waitUntilBroken(ros::Duration(5));
		}
	}
	void ModuleProxy::changeState(rexos_statemachine::State state) {
		if(state == rexos_statemachine::State::STATE_SAFE && getCurrentState() == rexos_statemachine::State::STATE_OFFLINE) {
			if(connectedWithNode == false) {
				moduleProxyListener->spawnNode(this);
				
				// wait for the node to come online
				if(connectedWithNode == false) {
					boost::unique_lock<boost::mutex> lock(nodeStartupMutex);
					nodeStartupCondition.wait(lock);
				}
			} else {
				REXOS_WARN("Node has already been stated, which is not expected (did someone manually start this node?)");
			}
		}
		StateMachineController::changeState(state);
	}
	void ModuleProxy::changeMode(rexos_statemachine::Mode mode) {
		StateMachineController::changeMode(mode);
	}
	void ModuleProxy::goToNextTransitionPhase(std::vector<rexos_module::CandidateModules> candidateModules) {
		rexos_module::TransitionResult result;
		result.candidates = candidateModules;
		transitionActionServer.setSucceeded(result);
		boost::unique_lock<boost::mutex> lock(transitionPhaseMutex);
		allowedToContinue = true;
		transitionPhaseCondition.notify_one();
	}
	void ModuleProxy::bind() {
		std::string moduleNamespaceName = identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber();
		
		REXOS_INFO_STREAM("binding B on " << (equipletName + "/bond")<< " id " << moduleNamespaceName);
		bond = new rexos_bond::Bond(equipletName + "/bond", moduleNamespaceName, this);
		bond->start();
	}
	
	void ModuleProxy::onStateChange(rexos_statemachine::State newState, rexos_statemachine::State previousState) {
		moduleProxyListener->onModuleStateChanged(this, newState, previousState);
	}
	void ModuleProxy::onModeChange(rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode) {
		moduleProxyListener->onModuleModeChanged(this, newMode, previousMode);
	}
	void ModuleProxy::onBondCallback(rexos_bond::Bond* bond, Event event) {
		if(event == FORMED) {
			REXOS_INFO("Bond has been formed");
			connectedWithNode = true;
			nodeStartupCondition.notify_one();
		} else {
			REXOS_WARN("Bond has been broken");
			moduleProxyListener->onModuleDied(this);
			connectedWithNode = false;
			delete bond;
			bond = NULL;
		}
	}
	void ModuleProxy::onModuleTransitionGoalCallback(const rexos_module::TransitionGoalConstPtr& goal) {
		REXOS_INFO("Recieved a goal call");
		std::vector<rexos_datatypes::SupportedMutation> supportedMutations;
		for(int i = 0; i < goal->gainedSupportedMutations.size(); i++) {
			rexos_datatypes::SupportedMutation supportedMutation(goal->gainedSupportedMutations.at(i));
			supportedMutations.push_back(supportedMutation);
		}
		std::vector<rexos_datatypes::RequiredMutation> requiredMutations;
		for(int i = 0; i < goal->requiredMutationsRequiredForNextPhase.size(); i++) {
			rexos_datatypes::RequiredMutation requiredMutation(
					goal->requiredMutationsRequiredForNextPhase.at(i).mutation, 
					goal->requiredMutationsRequiredForNextPhase.at(i).isOptional);
			requiredMutations.push_back(requiredMutation);
		}
		moduleProxyListener->onModuleTransitionPhaseCompleted(this, supportedMutations, requiredMutations);
		
		boost::unique_lock<boost::mutex> lock(transitionPhaseMutex);
		while(allowedToContinue == false) transitionPhaseCondition.wait(lock);
		allowedToContinue = false;
	}
}