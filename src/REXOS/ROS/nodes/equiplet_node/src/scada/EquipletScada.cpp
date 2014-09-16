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
		equiplet(equiplet){

	const char *mongooseOptions[] = {
			"listening_ports", std::to_string(port).c_str(),
			"document_root", SCADA_DEFAULT_DOCUMENT_ROOT,
			NULL
	};

	mg_callbacks mongooseCallbacks;
	memset(&mongooseCallbacks, 0, sizeof(mongooseCallbacks));
	mongooseCallbacks.begin_request = &EquipletScada::__mongooseBeginRequestCallback;

	mongooseContext = mg_start(&mongooseCallbacks, this, mongooseOptions);
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
	
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(
			moduleManufacturer, moduleTypeNumber, moduleSerialNumber);

	ModuleProxy* moduleProxy = moduleRegistry->getModule(moduleIdentifier);
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

void EquipletScada::mongooseProcessEquipletInfo(mg_connection* conn, mg_request_info* request_info) {
	const char* state = rexos_statemachine::state_txt[equiplet->getCurrentState()];
	const char* mode = rexos_statemachine::mode_txt[equiplet->getCurrentMode()];
	JSONNode jsonObject;
	jsonObject.push_back(JSONNode("name", equiplet->getEquipletName()));
	jsonObject.push_back(JSONNode("state", state));
	jsonObject.push_back(JSONNode("mode", mode));

	mg_printf(conn, "%s", ajax_reply_start_success);

	mg_printf(conn, "%s", jsonObject.write_formatted().c_str());
}

void EquipletScada::mongooseProcessModuleInfo(mg_connection* conn, mg_request_info* request_info) {
	JSONNode jsonModules(JSON_ARRAY);
	jsonModules.set_name("modules");

	std::vector<ModuleProxy*> proxies = moduleRegistry->getRegisteredModules();
	for (ModuleProxy* proxy : proxies) {
		JSONNode jsonModule;

		if(proxy == NULL){
			REXOS_ERROR("ModuleRegistry returns a NULL pointer");
			continue;
		}

		JSONNode jsonIdentifier = proxy->getModuleIdentifier().toJSONObject();
		jsonIdentifier.set_name("identifier");
		jsonModule.push_back(jsonIdentifier);
		jsonModule.push_back(JSONNode("mode", rexos_statemachine::mode_txt[proxy->getCurrentMode()]));
		jsonModule.push_back(JSONNode("state", rexos_statemachine::state_txt[proxy->getCurrentState()]));
		jsonModules.push_back(jsonModule);
	}

	JSONNode jsonObject;
	jsonObject.push_back(jsonModules);

	mg_printf(conn, "%s", ajax_reply_start_success);

	mg_printf(conn, "%s", jsonObject.write_formatted().c_str());
}

} /* namespace equiplet_node */
} /* namespace scada */
