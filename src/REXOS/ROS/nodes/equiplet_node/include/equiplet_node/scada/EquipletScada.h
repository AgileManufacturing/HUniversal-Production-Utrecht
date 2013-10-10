#pragma once

#include <equiplet_node/ModuleRegistry.h>
#include <equiplet_node/scada/mongoose.h>

#define SCADA_DEFAULT_DOCUMENT_ROOT "./src/www/equiplet_scada/"

namespace equiplet_node {

class EquipletNode;

namespace scada {

class EquipletScada {
public:
	EquipletScada(EquipletNode* equiplet, ModuleRegistry* moduleRegistry, int port = 8081);
	virtual ~EquipletScada();

private:
	static int __mongooseBeginRequestCallback(mg_connection* connection);
	int mongooseBeginRequestCallback(mg_connection* connection);

	void mongooseProcessChangeModuleMode(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessChangeEquipletMode(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessChangeEquipletState(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessEquipletInfo(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessModuleInfo(mg_connection* conn, mg_request_info* request_info);

	ModuleRegistry* moduleRegistry;
	EquipletNode* equiplet;

	mg_context* mongooseContext;
};

} /* namespace scada */
} /* namespace equiplet_node */
