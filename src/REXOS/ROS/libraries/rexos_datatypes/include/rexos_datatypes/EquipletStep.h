/* 
 * File:   EquipletStep.h
 * Author: alexander-ubuntu
 *
 * Created on June 16, 2013, 3:33 PM
 */
#ifndef EQUIPLETSTEP_H
#define	EQUIPLETSTEP_H

#include <string.h>
#include <iostream>
#include <map>
#include <utility>
#include "rexos_datatypes/InstructionData.h"
#include "rexos_datatypes/TimeData.h"

// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>

namespace rexos_datatypes{
    
    class EquipletStep {
    public:
        EquipletStep(JSONNode n);
        virtual ~EquipletStep();

        std::string getId();
        void setId(std::string id);

        std::string getServiceStepID();
        void setServiceStepID(std::string serviceStepID);

        std::string getNextStep();
        void setNextStep(std::string nextStep);

        int getModuleId();
        void setModuleId(int id);

        InstructionData getInstructionData();
        void setInstructionData(InstructionData instructionData);

        std::string getStatus();
        void setStatus(std::string status);

        std::map<std::string, std::string> getStatusData();
        void setStatusData(map<std::string, std::string> statusData);

        TimeData getTimeData();
        void setTimeData(TimeData timeData);

        JSONNode getJsonNode();
        std::string toJSONString();
    private:
            JSONNode jsonNode;
            std::string _id;
            std::string serviceStepID;
            std::string nextStep;
            int moduleId;
            InstructionData instructionData;
            std::string status;
            std::map<std::string, std::string> statusData;
            TimeData timeData;
            void setValues(const JSONNode & n);
            InstructionData setInstructionDataFromNode(const JSONNode & n);
            TimeData setTimeDataFromNode(const JSONNode & n);
            std::map<std::string, std::string> setMapFromNode(const JSONNode & n);
    };
}
#endif	/* EQUIPLETSTEP_H */

