#pragma once

#include <mongo/client/dbclient.h>

#include <memory>

#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

#define MONGODB_HOST "145.89.191.131"

class MOSTDatabaseClient {
public:
	struct ModuleData {
		int id;
		int state;
		int mode;

		ModuleData(int id, rexos_statemachine::State state, rexos_statemachine::Mode mode) :
				id(id), state(state), mode(mode) {
		}

		ModuleData() :
				id(-1), state(-1), mode(-1) {
		}

		static ModuleData fromBSON(const mongo::BSONObj& obj) {
			ModuleData result;
			result.id = obj.getIntField("id");
			result.state = obj.getIntField("state");
			result.mode =  obj.getIntField("mode");
			return result;
		}

		mongo::BSONObj toBSON() const {
			return BSON("id" << id << "state" << state << "mode" << mode);
		}
	};

	MOSTDatabaseClient() {
		connection.connect(MONGODB_HOST);
	}

	void setSafetyState(rexos_statemachine::State state){
		connection.update("most.equiplet", mongo::Query(), BSON("$set" << BSON("safety" << state)));
	}

	int getSafetyState(){
		std::unique_ptr<mongo::DBClientCursor> cursor = connection.query("most.equiplet", mongo::Query());
		if(cursor->more()){
			auto entry = cursor->next();
			if(entry.hasField("safety"))
				return entry.getIntField("safety");
		}

		return -1;
	}

	int getOperationalState(){
		std::unique_ptr<mongo::DBClientCursor> cursor = connection.query("most.equiplet", mongo::Query());
		if(cursor->more()){
			auto entry = cursor->next();
			if(entry.hasField("operational"))
				return entry.getIntField("operational");
		}

		return -1;
	}

	ModuleData getModuleData(int moduleID) {
		std::unique_ptr<mongo::DBClientCursor> cursor = connection.query("most.modules", QUERY("id" << moduleID));
		if(cursor->more()) {
			mongo::BSONObj p = cursor->next();
			return ModuleData::fromBSON(p);
		}
		return ModuleData();
	}

	void setModuleData(const ModuleData& data) {
		connection.update("most.modules", BSON("id" << data.id), data.toBSON(), true);
	}

	void clearModuleData() {
		connection.remove("most.modules", mongo::Query());
	}

	void sendEquipletCommand(std::string command, mongo::BSONObj args = mongo::BSONObj()){
		connection.insert("most.equipletCommands", BSON("command" << command << "args" << args));
	}

	std::vector<ModuleData> getAllModuleData() {
		std::vector<ModuleData> result;
		std::unique_ptr<mongo::DBClientCursor> cursor = connection.query("most.modules");
		while(cursor->more()){
			result.push_back(ModuleData::fromBSON(cursor->next()));
		}
		return result;
	}

private:
	mongo::DBClientConnection connection;
};
