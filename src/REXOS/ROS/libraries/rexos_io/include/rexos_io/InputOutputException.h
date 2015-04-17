#pragma once

#include <stdexcept>
#include <string>

namespace rexos_io{
	class InputOutputException : public std::runtime_error{
	public:
		InputOutputException(std::string message) : std::runtime_error(message){
		}
	};
}
