#pragma once

#include <equiplet_node/ModuleRegistry.h>
#include <equiplet_node/scada/mongoose.h>

namespace equiplet_node {
namespace scada {

class EquipletScada {
public:
	EquipletScada(ModuleRegistry* moduleRegistry, int port = 8081);
	virtual ~EquipletScada();

private:
	static int __mongooseBeginRequestCallback(mg_connection* connection);
	int mongooseBeginRequestCallback(mg_connection* connection);

	ModuleRegistry* moduleRegistry;

	mg_context* mongooseContext;
};

} /* namespace scada */
} /* namespace equiplet_node */
