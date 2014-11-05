#include <equiplet_node/CyclicDependencyException.h>

namespace equiplet_node
{

CyclicDependencyException::CyclicDependencyException(std::vector<std::vector<rexos_datatypes::TransitionPhase>> partlyResolvedGraph) :
	std::runtime_error("Unable to fully resolve graph due to cyclic depenency"), partlyResolvedGraph(partlyResolvedGraph)
{
	// nothing to do here
}
CyclicDependencyException::~CyclicDependencyException() throw() {
	// nothing to do here
}
std::vector<std::vector<rexos_datatypes::TransitionPhase>> CyclicDependencyException::getPartlyResolvedGraph() {
	return partlyResolvedGraph;
}
}
