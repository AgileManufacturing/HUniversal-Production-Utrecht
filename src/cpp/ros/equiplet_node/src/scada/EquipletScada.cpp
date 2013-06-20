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
	const struct mg_request_info *request_info = mg_get_request_info(connection);
	char content[100];

	// Prepare the message we're going to send
	int content_length = snprintf(content, sizeof(content),
			"Hello from mongoose! Remote port: %d", request_info->remote_port);

	// Send HTTP reply to the client
	mg_printf(connection, "HTTP/1.1 200 OK\r\n"
			"Content-Type: text/plain\r\n"
			"Content-Length: %d\r\n" // Always set Content-Length
			"\r\n"
			"%s", content_length, content);

	// Returning non-zero tells mongoose that our function has replied to
	// the client, and mongoose should not send client any more data.
	return 1;
}

} /* namespace equiplet_node */
} /* namespace scada */
