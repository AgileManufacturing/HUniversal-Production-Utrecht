/*
 * EquipletScada.cpp
 *
 *  Created on: Jun 20, 2013
 *      Author: joris
 */

#include <equiplet_node/scada/EquipletScada.h>
#include <equiplet_node/EquipletNode.h>

#include <algorithm>
#include <string>

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>

#ifdef __CDT_PARSER__
namespace std {
std::string to_string(...);
}
#endif

namespace equiplet_node {
namespace scada {

static const char *ajax_reply_start_success = "HTTP/1.1 200 OK\r\n"
		"Cache: no-cache\r\n"
		"Content-Type: application/x-javascript\r\n"
		"\r\n";

static const char *ajax_reply_start_failed =
		"HTTP/1.1 500 Internal Server Error\r\n"
				"Cache: no-cache\r\n"
				"Content-Type: application/x-javascript\r\n"
				"\r\n";


EquipletScada::EquipletScada(EquipletNode* equiplet, ModuleRegistry* moduleRegistry, int port) :
		moduleRegistry(moduleRegistry),
		equiplet(equiplet),
		humanInteractionActionServer(equiplet->getNodeHandle(), equiplet->getEquipletName() + "/humanInteraction/", 
			boost::bind(&EquipletScada::onHumanInteractionAction, this, _1), false),
		humanInteractionFormJson(""){

	const char *mongooseOptions[] = {
			"listening_ports", std::to_string(port).c_str(),
			"document_root", SCADA_DEFAULT_DOCUMENT_ROOT,
			NULL
	};

	mg_callbacks mongooseCallbacks;
	memset(&mongooseCallbacks, 0, sizeof(mongooseCallbacks));
	mongooseCallbacks.begin_request = &EquipletScada::__mongooseBeginRequestCallback;

	mongooseContext = mg_start(&mongooseCallbacks, this, mongooseOptions);
	humanInteractionActionServer.start();
	REXOS_INFO_STREAM("humanInteractionActionServer has been started at " << (equiplet->getEquipletName() + "/humanInteraction/"));
}

EquipletScada::~EquipletScada() {
	mg_stop (mongooseContext);
}

int EquipletScada::__mongooseBeginRequestCallback(mg_connection* connection) {
	EquipletScada* thiz = (EquipletScada*)mg_get_request_info(connection)->user_data;
	return thiz->mongooseBeginRequestCallback(connection);
}
int EquipletScada::mongooseBeginRequestCallback(mg_connection* connection) {
	mg_request_info *request_info = mg_get_request_info(connection);

	int processed = 1;
	if (strcmp(request_info->uri, "/remote/equipletInfo") == 0) {
		mongooseProcessEquipletInfo(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/moduleInfo") == 0) {
		mongooseProcessModuleInfo(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/changeModuleMode") == 0) {
		mongooseProcessChangeModuleMode(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/changeEquipletMode") == 0) {
		mongooseProcessChangeEquipletMode(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/changeEquipletState") == 0) {
		mongooseProcessChangeEquipletState(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/humanInteractionGet") == 0) {
		mongooseProcessHumanInteractionGet(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/humanInteractionSubmit") == 0) {
		mongooseProcessHumanInteractionResult(connection, request_info);
	} else {
		processed = 0;
	}

	return processed;
}

void EquipletScada::mongooseProcessChangeModuleMode(mg_connection* conn, mg_request_info* request_info) {
	char moduleManufacturer[201], moduleTypeNumber[201], moduleSerialNumber[201], moduleModi[64];
	const char* query = request_info->query_string;
	const size_t query_len = strlen(query);

	mg_get_var(query, query_len, "manufacturer", moduleManufacturer, sizeof(moduleManufacturer));
	mg_get_var(query, query_len, "typeNumber", moduleTypeNumber, sizeof(moduleTypeNumber));
	mg_get_var(query, query_len, "serialNumber", moduleSerialNumber, sizeof(moduleSerialNumber));
	mg_get_var(query, query_len, "mode", moduleModi, sizeof(moduleModi));
	
	rexos_datatypes::ModuleIdentifier moduleIdentifier(moduleManufacturer, moduleTypeNumber, moduleSerialNumber);

	rexos_module::ModuleProxy* moduleProxy = moduleRegistry->getModule(moduleIdentifier);
	if(moduleProxy == NULL) {
		mg_printf(conn, "%s", ajax_reply_start_failed);
		return;
	}

	for(int i = 0; i < MODE_COUNT; i++){
		std::string testModi = moduleModi;
		std::transform(testModi.begin(), testModi.end(), testModi.begin(), ::tolower);
		std::replace(testModi.begin(), testModi.end(),' ', '_');

		std::string moduleModeStr = rexos_statemachine::mode_txt[i];
		std::transform(moduleModeStr.begin(), moduleModeStr.end(), moduleModeStr.begin(), ::tolower);
		std::replace(moduleModeStr.begin(), moduleModeStr.end(),' ', '_');

		if(testModi == moduleModeStr){
			moduleProxy->changeMode(static_cast<rexos_statemachine::Mode>(i));
			mg_printf(conn, "%s", ajax_reply_start_success);
			return;
		}
	}

	mg_printf(conn, "%s", ajax_reply_start_failed);
}

void EquipletScada::mongooseProcessChangeEquipletMode(mg_connection* conn, mg_request_info* request_info) {
	char equipletMode[64];
	const char* query = request_info->query_string;
	const size_t query_len = strlen(query);

	mg_get_var(query, query_len, "mode", equipletMode, sizeof(equipletMode));

	for(int i = 0; i < MODE_COUNT; i++){
		std::string testModi = equipletMode;
		std::transform(testModi.begin(), testModi.end(), testModi.begin(), ::tolower);
		std::replace(testModi.begin(), testModi.end(),' ', '_');

		std::string equipletModeStr = rexos_statemachine::mode_txt[i];
		std::transform(equipletModeStr.begin(), equipletModeStr.end(), equipletModeStr.begin(), ::tolower);
		std::replace(equipletModeStr.begin(), equipletModeStr.end(),' ', '_');

		if(testModi == equipletModeStr){
			actionlib::SimpleActionClient<rexos_statemachine::ChangeModeAction> client(equiplet->getNodeHandle(), equiplet->getEquipletName() + "/change_mode");
			rexos_statemachine::ChangeModeGoal goal;
			goal.desiredMode = static_cast<rexos_statemachine::Mode>(i);
			client.sendGoal(goal);

			mg_printf(conn, "%s", ajax_reply_start_success);
			return;
		}
	}

	mg_printf(conn, "%s", ajax_reply_start_failed);
}

void EquipletScada::mongooseProcessChangeEquipletState(mg_connection* conn, mg_request_info* request_info) {
	char equipletState[64];
	const char* query = request_info->query_string;
	const size_t query_len = strlen(query);

	mg_get_var(query, query_len, "state", equipletState, sizeof(equipletState));

	for(int i = 0; i < STATE_COUNT; i++){
		std::string testState = equipletState;
		std::transform(testState.begin(), testState.end(), testState.begin(), ::tolower);
		std::replace(testState.begin(), testState.end(),' ', '_');

		std::string equipletStateStr = rexos_statemachine::state_txt[i];
		std::transform(equipletStateStr.begin(), equipletStateStr.end(), equipletStateStr.begin(), ::tolower);
		std::replace(equipletStateStr.begin(), equipletStateStr.end(),' ', '_');

		if(testState == equipletStateStr){
			actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction> client(equiplet->getNodeHandle(), equiplet->getEquipletName() + "/change_state");
			rexos_statemachine::ChangeStateGoal goal;
			goal.desiredState = static_cast<rexos_statemachine::State>(i);
			client.sendGoal(goal);

			mg_printf(conn, "%s", ajax_reply_start_success);
			return;
		}
	}

	mg_printf(conn, "%s", ajax_reply_start_failed);
}

void EquipletScada::mongooseProcessHumanInteractionResult(mg_connection* conn, mg_request_info* request_info) {
	char humanInteractionResult[10024];
	const char* query = request_info->query_string;
	const size_t query_len = strlen(query);

	mg_get_var(query, query_len, "result", humanInteractionResult, sizeof(humanInteractionResult));
	
	humanInteractionMutex.lock();
	humanInteractionResultJson = humanInteractionResult;
	humanInteractionMutex.unlock();
	
	humanInteractionActionCondition.notify_one();
	
	mg_printf(conn, "%s", ajax_reply_start_success);
	
	return;
}
void EquipletScada::mongooseProcessEquipletInfo(mg_connection* conn, mg_request_info* request_info) {
	const char* state = rexos_statemachine::state_txt[equiplet->getCurrentState()];
	const char* mode = rexos_statemachine::mode_txt[equiplet->getCurrentMode()];
	Json::Value jsonObject;
	jsonObject["name"] = equiplet->getEquipletName();
	jsonObject["state"] = state;
	jsonObject["mode"] = mode;

	mg_printf(conn, "%s", ajax_reply_start_success);

	Json::StyledWriter writer;
	mg_printf(conn, "%s", writer.write(jsonObject).c_str());
}

void EquipletScada::mongooseProcessModuleInfo(mg_connection* conn, mg_request_info* request_info) {
	Json::Value jsonModulesArray;

	std::vector<rexos_module::ModuleProxy*> proxies = moduleRegistry->getRegisteredModules();
	for (rexos_module::ModuleProxy* proxy : proxies) {
		Json::Value jsonModule;

		if(proxy == NULL){
			REXOS_ERROR("ModuleRegistry returns a NULL pointer");
			continue;
		}

		Json::Value jsonIdentifier = proxy->getModuleIdentifier().toJSONObject();
		jsonModule["identifier"] = jsonIdentifier;
		jsonModule["mode"] = rexos_statemachine::mode_txt[proxy->getCurrentMode()];
		jsonModule["state"] = rexos_statemachine::state_txt[proxy->getCurrentState()];
		jsonModulesArray.append(jsonModule);
	}

	Json::Value jsonObject;
	jsonObject["modules"] = jsonModulesArray;

	mg_printf(conn, "%s", ajax_reply_start_success);
	
	Json::StyledWriter writer;
	mg_printf(conn, "%s", writer.write(jsonObject).c_str());
}

void EquipletScada::mongooseProcessHumanInteractionGet(mg_connection* conn, mg_request_info* request_info) {
	mg_printf(conn, "%s", ajax_reply_start_success);

	humanInteractionMutex.lock();
	std::string message = humanInteractionFormJson;
	ROS_DEBUG_STREAM("human interaction get " << message);
	humanInteractionFormJson = "";
	humanInteractionMutex.unlock();
	
	mg_printf(conn, "%s", message.c_str());
}
void EquipletScada::onHumanInteractionAction(const HumanInteractionGoalConstPtr& goal) {
	ROS_ERROR("human interaction goal recieved");
	humanInteractionMutex.lock();
	humanInteractionFormJson = goal->humanInteractionFormJson;
	humanInteractionMutex.unlock();
	ROS_INFO("human interaction: waiting for result");
	
	boost::unique_lock<boost::mutex> lock(humanInteractionActionMutex);
	humanInteractionActionCondition.wait(lock);
	
	ROS_INFO("human interaction  acquiring result");
	humanInteractionMutex.lock();
	std::string resultJson = humanInteractionResultJson;
	HumanInteractionResult result;
	result.humanInteractionResult = resultJson;
	humanInteractionMutex.unlock();
	
	humanInteractionActionServer.setSucceeded(result);
	ROS_INFO("human interaction sent result");
}
} /* namespace equiplet_node */
} /* namespace scada */
