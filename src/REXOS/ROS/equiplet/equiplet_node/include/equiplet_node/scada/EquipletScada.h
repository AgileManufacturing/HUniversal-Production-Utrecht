#pragma once

#include <equiplet_node/ModuleRegistry.h>
#include <equiplet_node/scada/mongoose.h>
#include <rexos_logger/rexos_logger.h>


#include <equiplet_node/HumanInteractionAction.h>

#define SCADA_DEFAULT_DOCUMENT_ROOT "./src/REXOS/www/equiplet_scada/"

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
	void mongooseProcessHumanInteractionResult(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessEquipletInfo(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessModuleInfo(mg_connection* conn, mg_request_info* request_info);
	void mongooseProcessHumanInteractionGet(mg_connection* conn, mg_request_info* request_info);

	ModuleRegistry* moduleRegistry;
	EquipletNode* equiplet;

	mg_context* mongooseContext;
	
	actionlib::SimpleActionServer<HumanInteractionAction> humanInteractionActionServer;
	void onHumanInteractionAction(const HumanInteractionGoalConstPtr& goal);
	std::string humanInteractionFormJson;
	std::string humanInteractionResultJson;
	boost::mutex humanInteractionMutex;
	boost::condition_variable humanInteractionActionCondition;
	boost::mutex humanInteractionActionMutex;
};

} /* namespace scada */
} /* namespace equiplet_node */
