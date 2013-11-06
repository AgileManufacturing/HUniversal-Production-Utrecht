#pragma once

#include <stdexcept>

namespace rexos_knowledge_database{
	class KnowledgeDatabaseException : public std::runtime_error{
	public:
		KnowledgeDatabaseException(const char* message);
	};
}