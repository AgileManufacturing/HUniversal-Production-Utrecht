#include <rexos_knowledge_database/KnowledgeDatabaseException.h>

namespace rexos_knowledge_database{
	KnowledgeDatabaseException::KnowledgeDatabaseException(const char* message) : 
			std::runtime_error(message)
	{
		
	}
}