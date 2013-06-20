/*
 * EquipletScada.cpp
 *
 *  Created on: Jun 20, 2013
 *      Author: joris
 */

#include <equiplet_node/scada/EquipletScada.h>

#ifdef __CDT_PARSER__
namespace std {
std::string to_string(...);
}
#endif

namespace equiplet_node {
namespace scada {

EquipletScada::EquipletScada(ModuleRegistry* moduleRegistry, int port) :
		moduleRegistry(moduleRegistry) {

	const char *mongooseOptions[] = {
			"listening_ports", std::to_string(port).c_str(),
			"document_root", SCADA_DEFAULT_DOCUMENT_ROOT,
			NULL
	};

	mg_callbacks mongooseCallbacks;
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

}
void EquipletScada::mongooseProcessMakeEquipletSafe(mg_connection* conn, mg_request_info* request_info) {

}
void EquipletScada::mongooseProcessEquipletInfo(mg_connection* conn, mg_request_info* request_info) {

}
void EquipletScada::mongooseProcessModuleInfo(mg_connection* conn, mg_request_info* request_info) {

}

} /* namespace equiplet_node */
} /* namespace scada */
