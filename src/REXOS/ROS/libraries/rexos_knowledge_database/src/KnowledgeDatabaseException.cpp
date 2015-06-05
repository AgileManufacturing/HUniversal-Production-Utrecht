#include <rexos_knowledge_database/KnowledgeDatabaseException.h>

namespace rexos_knowledge_database{
	KnowledgeDatabaseException::KnowledgeDatabaseException(std::string message) : 
			std::runtime_error(message.c_str())
	{
		
	}
}