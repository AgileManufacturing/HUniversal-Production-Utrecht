#include <rexos_configuration/Configuration.h>

#include <jsoncpp/json/reader.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

namespace rexos_configuration {
	Json::Value Configuration::configuration = Json::Value::null;
	
	Json::Value Configuration::getProperty(std::string path, std::string equipletName) {
		if(configuration == Json::Value::null) initialize();
		Json::Value currentObject = configuration;
		
		std::vector<std::string> pathSegments;
		boost::trim_if(path, boost::is_any_of("/"));
		boost::split(pathSegments, path, boost::is_any_of("/"));
		for(std::string pathSegment : pathSegments) {
			if(currentObject.isMember(pathSegment) == true) {
				currentObject = currentObject[pathSegment];
			} else {
				throw std::runtime_error("rexos_configuration: Unable to retrieve property in configuration, using path " + path);
			}
		}
		
		if(currentObject.isMember(equipletName) == true) {
			return currentObject[equipletName];
		} else {
			return currentObject[DEFAULT_PROPERTY_NAME];
		}
	}
	
	void Configuration::initialize() {
		Json::Reader reader;
		std::ifstream settingsFile(DEFAULT_PROPERTIES_FILE_PATH, std::ifstream::binary);
		
		if(reader.parse(settingsFile, configuration) == false) {
			throw std::runtime_error("rexos_configuration: Parsing of settings file failed");
		}
		settingsFile.close();
	}
}
