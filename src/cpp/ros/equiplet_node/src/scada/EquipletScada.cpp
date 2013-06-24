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
	} else if (strcmp(request_info->uri, "/remote/changeModuleModi") == 0) {
		mongooseProcessChangeModuleModi(connection, request_info);
	} else if (strcmp(request_info->uri, "/remote/makeEquipletSafe") == 0) {
		mongooseProcessMakeEquipletSafe(connection, request_info);
	} else {
		processed = 0;
	}

	return processed;

	return 1;
}

void EquipletScada::mongooseProcessChangeModuleModi(mg_connection* conn, mg_request_info* request_info) {
	char moduleId[64], moduleModi[64];
	const char* query = request_info->query_string;
	const size_t query_len = strlen(query);

	mg_get_var(query, query_len, "id", moduleId, sizeof(moduleId));
	mg_get_var(query, query_len, "modi", moduleModi, sizeof(moduleModi));

	ModuleProxy* moduleProxy = moduleRegistry->getModule(atoi(moduleId));
	if(moduleProxy == NULL) {
		mg_printf(conn, "%s", ajax_reply_start_failed);
		return;
	}

	for(int i = 0; i < MODE_COUNT; i++){
		std::string testModi = moduleModi;
		std::transform(testModi.begin(), testModi.end(), testModi.begin(), ::tolower);
		std::replace(testModi.begin(), testModi.end(),' ', '_');

		if(testModi == moduleModi){
			moduleProxy->changeMode(static_cast<rexos_statemachine::Mode>(i));
			mg_printf(conn, "%s", ajax_reply_start_success);
			return;
		}
	}

	mg_printf(conn, "%s", ajax_reply_start_failed);
}

void EquipletScada::mongooseProcessMakeEquipletSafe(mg_connection* conn, mg_request_info* request_info) {
	actionlib::SimpleActionClient<rexos_statemachine::ChangeModeAction> client(equiplet->getNodeHandle(), equiplet->getName() + "change_mode");
	rexos_statemachine::ChangeModeGoal goal;
	goal.desiredMode = rexos_statemachine::Mode::MODE_CRITICAL_ERROR;
	client.sendGoal(goal);

	mg_printf(conn, "%s", ajax_reply_start_success);
}

void EquipletScada::mongooseProcessEquipletInfo(mg_connection* conn, mg_request_info* request_info) {
	const char* state = rexos_statemachine::state_txt[equiplet->getCurrentState()];
	const char* mode = rexos_statemachine::mode_txt[equiplet->getCurrentMode()];
	JSONNode jsonObject;
	jsonObject.push_back(JSONNode("name", equiplet->getName()));
	jsonObject.push_back(JSONNode("state", state));
	jsonObject.push_back(JSONNode("mode", mode));

	mg_printf(conn, "%s", ajax_reply_start_success);

	mg_printf(conn, "%s", jsonObject.write_formatted().c_str());
}

void EquipletScada::mongooseProcessModuleInfo(mg_connection* conn, mg_request_info* request_info) {
	JSONNode jsonModules(JSON_ARRAY);
	jsonModules.set_name("modules");

	std::vector<ModuleProxy*>::iterator it;
	for (it = moduleRegistry->getRegisteredModules().begin(); it != moduleRegistry->getRegisteredModules().end(); it++) {
		JSONNode jsonModule;

		jsonModule.push_back(JSONNode("id", (*it)->getModuleId()));
		jsonModule.push_back(JSONNode("modi", (*it)->getCurrentMode()));
		jsonModule.push_back(JSONNode("state", (*it)->getCurrentState()));
		jsonModule.push_back(JSONNode("name", (*it)->getModuleNodeName()));
		jsonModules.push_back(jsonModule);
	}

	JSONNode jsonObject;
	jsonObject.push_back(jsonModules);

	mg_printf(conn, "%s", ajax_reply_start_success);

	mg_printf(conn, "%s", jsonObject.write_formatted().c_str());
}

} /* namespace equiplet_node */
} /* namespace scada */
