#pragma once

#include <stdexcept>
#include <string>

namespace rexos_knowledge_database{
	class KnowledgeDatabaseException : public std::runtime_error{
	public:
		KnowledgeDatabaseException(std::string message);
	};
}