#include <part_locator_node/part_locator_node.h>

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>
#include <algorithm>
#include <vector>

#include <environment_cache/EnvironmentCache.h>
#include <rexos_module/ModuleInterface.h>
#include <rexos_utilities/Utilities.h>
#include <rexos_datatypes/EquipletStep.h>

namespace part_locator_node {
	const Vector2 PartLocatorNode::EXPECTED_DIRECTION = Vector2(-1, 0);
	const Vector2 PartLocatorNode::EXPECTED_ITEM_DIRECTION = Vector2(-1, 0);
	const int PartLocatorNode::minCornerSamples = 11;
	const int PartLocatorNode::minItemSamples = 11;

	PartLocatorNode::PartLocatorNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier):
			rexos_module::Module(equipletName, moduleIdentifier),
			environmentCacheClient(rexos_module::AbstractModule::nodeHandle.serviceClient<environment_cache::setData>("setData")),
			samplesTopLeft(minCornerSamples),
			samplesTopRight(minCornerSamples),
			samplesBottomRight(minCornerSamples)
	{
		REXOS_INFO("Constructing");
		
		std::string properties = this->getModuleTypeProperties();
		
		Json::Reader reader;
		Json::Value jsonNode;
		reader.parse(properties, jsonNode);
		
		topLeftValue = jsonNode["topLeftValue"].asString();
		REXOS_INFO_STREAM("found topLeftValue " << topLeftValue);
		topRightValue = jsonNode["topRightValue"].asString();
		REXOS_INFO_STREAM("found topRightValue " << topRightValue);
		bottomRightValue = jsonNode["bottomRightValue"].asString();
		REXOS_INFO_STREAM("found bottomRightValue " << bottomRightValue);
		workPlaneWidth = jsonNode["workPlaneWidth"].asDouble();
		REXOS_INFO_STREAM("found workPlaneWidth " << workPlaneWidth);
		workPlaneHeight = jsonNode["workPlaneHeight"].asDouble();
		REXOS_INFO_STREAM("found workPlaneHeight " << workPlaneHeight);
		
		if(std::isnan(workPlaneWidth) || std::isnan(workPlaneHeight) || 
				topLeftValue.length() == 0 || topRightValue.length() == 0 || bottomRightValue.length() == 0) {
			throw std::runtime_error("The properties do not contain the top/bottom left/right values or do not contain the workplane width/height");
		}
	}

	void PartLocatorNode::qrCodeCallback(const vision_node::QrCodes & message) {
		detectCorners(message);
		
		// can not do conversion without the corners
		if(
			samplesTopLeft.size() < minCornerSamples || 
			samplesTopRight.size() < minCornerSamples || 
			samplesBottomRight.size() < minCornerSamples) {
			return;
		}
		
		int collectionSize = message.qrCodes.size();
		for(int i = 0; i < collectionSize; i++){
			Vector2* points = new Vector2[3];
			for(int j = 0; j < 3; j++){
				Vector3 oldCoor;

				oldCoor.x = message.qrCodes[i].corners[j].x;
				oldCoor.y = message.qrCodes[i].corners[j].y;
				oldCoor.z = 1;
			
				//REXOS_DEBUG_STREAM("QrCode \t" << message.qrCodes[i].value << " corner \t" << j);
				//if(j == 1) REXOS_INFO_STREAM("value: " << message.qrCodes[i].value);
				//if(j == 1) REXOS_INFO_STREAM("oldCoor: " << oldCoor);
				Vector3 newCoor = totalMatrix * oldCoor;
				//if(j == 1) REXOS_INFO_STREAM("newCoor: " << newCoor);
				newCoor = postCorrectionTotalMatrix * newCoor;
				//if(j == 1) REXOS_INFO_STREAM("newCoor: " << newCoor);
				//if(j == 1) REXOS_INFO_STREAM("eqlCoor: " << convertToEquipletCoordinate(newCoor));
				/*if(message.qrCodes[i].value == "GC4x4MB_1") {
					REXOS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\toldCoor \t" << oldCoor << "newCoor \t" << newCoor);
				}*/

				points[j] = Vector2(newCoor.x, newCoor.y);
			}
			Vector2 lineA2B = points[1] - points[0];
			Vector2 lineB2C = points[2] - points[1];
			
			//REXOS_DEBUG_STREAM("lineA2B \t" << lineA2B << " length " << lineA2B.length());
			//REXOS_DEBUG_STREAM("lineB2C \t" << lineB2C << " length " << lineB2C.length());
			
			QrCode qrCode;
			qrCode.location = Vector2(
				points[0].x + (lineA2B / 2).x + (lineB2C / 2).x,
				points[0].y + (lineA2B / 2).y + (lineB2C / 2).y
			);
			qrCode.angle = getItemRotationAngle(lineA2B);
			
			QrCode smoothCenterCoor = calculateSmoothPos(message.qrCodes[i].value, qrCode);
			
			Vector3 equipletCoor = convertToEquipletCoordinate(Vector3(smoothCenterCoor.location.x, smoothCenterCoor.location.y, 0));
			/*if(message.qrCodes[i].value == "GC4x4MB_1") {
				REXOS_DEBUG_STREAM("-equipletCoor \t" << equipletCoor);
			}*/
			
			storeInEnviromentCache(message.qrCodes[i].value, equipletCoor, qrCode.angle);
			
			delete[] points;
		}
	}
	PartLocatorNode::QrCode PartLocatorNode::calculateSmoothPos(std::string name, PartLocatorNode::QrCode lastPosition) {
		boost::circular_buffer<QrCode> buffer;
		try{
			buffer = smoothBuffer.at(name);
		} catch(std::out_of_range ex) {
			buffer = boost::circular_buffer<QrCode>(minItemSamples);
		}
		buffer.push_back(lastPosition);
		
		return calculateSmoothPos(buffer);
	}
	PartLocatorNode::QrCode PartLocatorNode::calculateSmoothPos(boost::circular_buffer<PartLocatorNode::QrCode> buffer) {
		QrCode* points = buffer.linearize();
		std::vector<double> xArray;
		std::vector<double> yArray;
		std::vector<double> angleArray;
		for(int i = 0; i < buffer.size(); i++) {
			xArray.push_back(points[i].location.x);
			yArray.push_back(points[i].location.y);
			angleArray.push_back(points[i].angle);
		}
		
		double sumX = std::accumulate(xArray.begin(), xArray.end(), 0.0);
		double sumY = std::accumulate(yArray.begin(), yArray.end(), 0.0);
		double sumAngle = std::accumulate(angleArray.begin(), angleArray.end(), 0.0);
		
		QrCode output;
		output.location = Vector2(sumX / buffer.size(), sumY / buffer.size());
		output.angle = sumAngle / buffer.size();
		return output;
	}

	void PartLocatorNode::detectCorners(const vision_node::QrCodes & message) {
		REXOS_DEBUG_STREAM("currentTopLeftCoor " << currentTopLeftCoor.location);
		REXOS_DEBUG_STREAM("currentTopRightCoor " << currentTopRightCoor.location);
		REXOS_DEBUG_STREAM("currentBottomRightCoor " << currentBottomRightCoor.location);
		
		bool updateMatrices = false;
		for(int i = 0; i < message.qrCodes.size(); i++) {
			QrCode qrCode;
			qrCode.location.x = message.qrCodes[i].corners[1].x;
			qrCode.location.y = message.qrCodes[i].corners[1].y;
			
			if(topLeftValue.compare(message.qrCodes[i].value) == 0){
				samplesTopLeft.push_back(qrCode);
				if(samplesTopLeft.size() == minCornerSamples - 1) {
					originalTopLeftCoor = calculateSmoothPos(samplesTopLeft);
				} else if(samplesTopLeft.size() == minCornerSamples) {
					currentTopLeftCoor = calculateSmoothPos(samplesTopLeft);
					updateMatrices = true;
				}
			} else if(topRightValue.compare(message.qrCodes[i].value) == 0){
				samplesTopRight.push_back(qrCode);
				if(samplesTopRight.size() == minCornerSamples - 1) {
					originalTopRightCoor = calculateSmoothPos(samplesTopRight);
				} else if(samplesTopRight.size() == minCornerSamples) {
					currentTopRightCoor = calculateSmoothPos(samplesTopRight);
					updateMatrices = true;
				}
			} else if(bottomRightValue.compare(message.qrCodes[i].value) == 0){
				samplesBottomRight.push_back(qrCode);
				if(samplesBottomRight.size() == minCornerSamples - 1) {
					originalBottomRightCoor = calculateSmoothPos(samplesBottomRight);
				} else if(samplesBottomRight.size() == minCornerSamples) {
					currentBottomRightCoor = calculateSmoothPos(samplesBottomRight);
					updateMatrices = true;
				}
			}
		}
		if(updateMatrices == true) this->updateMatrices();
	}
	double PartLocatorNode::getItemRotationAngle(Vector2 lineA2B) {
		Vector2 actualItemDirection(lineA2B);
		actualItemDirection.normalize();
		
		// calulate the expected angle (0)
		double expectedItemAngle = acos(EXPECTED_ITEM_DIRECTION.x);
		if(EXPECTED_ITEM_DIRECTION.y < 0) expectedItemAngle = 0 - expectedItemAngle;
		// calulate the actual angle (0)
		double actualItemAngle = acos(actualItemDirection.x);
		if(actualItemDirection.y < 0) actualItemAngle = 0 - actualItemAngle;
		
		double angle = actualItemAngle - expectedItemAngle;
		
		//REXOS_DEBUG_STREAM("-expectedItemAngle \t" << expectedItemAngle);
		//REXOS_DEBUG_STREAM("-actualItemAngle \t" << actualItemAngle);
		/*if(message.qrCodes[i].value == "GC4x4MB_1") {
			REXOS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\t-angle \t" << angle);
		}*/
		return angle;
	}
	void PartLocatorNode::storeInEnviromentCache(std::string value, Vector3 location, double angle) {
			environment_cache::setData serviceCall;
			serviceCall.request.identifier = value;
			
			Json::Value data;
			Json::Value locationJson;
			locationJson["x"] = location.x;
			locationJson["y"] = location.y;
			locationJson["z"] = location.z;
			Json::Value rotationJson;
			rotationJson["z"] = angle;
			
			data["location"] = locationJson;
			data["rotation"] = rotationJson;
			
			Json::StyledWriter writer;
			serviceCall.request.json = writer.write(data);
			environmentCacheClient.call(serviceCall);
	}

	void PartLocatorNode::updateMatrices() {
		totalMatrix = calculateScaleMatrix() * calculateRotationMatrix() * calculateOffsetMatrix();
		//REXOS_INFO_STREAM("totalMatrix " << totalMatrix);
		
	}
	Matrix3 PartLocatorNode::calculateOffsetMatrix() {
		////////////
		// calulate midpoint
		////////////
		// line between topLeft and topRight
		Vector2 lineTl2Tr;
		lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
		lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
		REXOS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
		
		// line between topRight and bottomRight
		Vector2 lineTr2Br;
		lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
		lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
		REXOS_DEBUG_STREAM("lineTr2Br " << lineTr2Br);
		
		// calulate new midpoint by deviding lineTl2Tr and bottomRight and then adding them
		Vector2 halfLineTl2Tr;
		halfLineTl2Tr.x = lineTl2Tr.x / 2;
		halfLineTl2Tr.y = lineTl2Tr.y / 2;
		Vector2 halfLineTr2Br;
		halfLineTr2Br.x = lineTr2Br.x / 2;
		halfLineTr2Br.y = lineTr2Br.y / 2;
		
		Vector2 midPoint;
		midPoint.x = currentTopLeftCoor.location.x + halfLineTl2Tr.x + halfLineTr2Br.x;
		midPoint.y = currentTopLeftCoor.location.y + halfLineTl2Tr.y + halfLineTr2Br.y;
		REXOS_DEBUG_STREAM("midpoint " << midPoint);
		
		Matrix3 translationMatrix;
		translationMatrix[2] = -midPoint.x;
		translationMatrix[5] = -midPoint.y;
		REXOS_DEBUG_STREAM("translationMatrix " << translationMatrix);
		
		return translationMatrix;
	}
	Matrix3 PartLocatorNode::calculateRotationMatrix() {
		////////////
		// calulate rotation angle
		////////////
		// line between topLeft and topRight
		Vector2 lineTl2Tr;
		lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
		lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
		REXOS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
		
		// the expected vector is horizontal to the left because TR is at the right of TL (eg. walk in this direction to get to TR)
		Vector2 actualDirection;
		actualDirection.x = lineTl2Tr.x;
		actualDirection.y = lineTl2Tr.y;
		actualDirection.normalize();
		
		// we could calulate the angle by calulating the dot product and convert it to radians. But this is a more fancy approach (it actually aint)
		// calulate the expected angle (0)
		double expectedAngle = acos(EXPECTED_DIRECTION.x);
		if(EXPECTED_DIRECTION.y < 0) expectedAngle = 0 - expectedAngle;
		
		// calulate the actual angle
		double actualAngle = acos(actualDirection.x);
		if(actualDirection.y < 0) actualAngle = 0 - actualAngle;
		
		// substract the two angles to get the correction angle
		double correctionAngle = expectedAngle - actualAngle;
		
		Matrix3 rotationMatrix;
		rotationMatrix[0] = cos(correctionAngle);
		rotationMatrix[1] = -sin(correctionAngle);
		rotationMatrix[3] = sin(correctionAngle);
		rotationMatrix[4] = cos(correctionAngle);
		REXOS_DEBUG_STREAM("rotationMatrix " << rotationMatrix);

		REXOS_DEBUG_STREAM("actualDirection " << actualDirection);
		REXOS_DEBUG_STREAM("correctionAngle " << correctionAngle);
		
		return rotationMatrix;
	}
	Matrix3 PartLocatorNode::calculateScaleMatrix() {
		////////////
		// scale to workplate coor system
		////////////
		// line between topLeft and topRight
		Vector2 lineTl2Tr;
		lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
		lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
		REXOS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
		
		// line between topRight and bottomRight
		Vector2 lineTr2Br;
		lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
		lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
		REXOS_DEBUG_STREAM("lineTr2Br " << lineTr2Br);
		

		double workplateWidth = 80;
		double workplateHeight = 80;
		Matrix3 scaleMatrix;
		scaleMatrix[0] = -(	(workplateWidth / 1) / (lineTl2Tr.length()));
		scaleMatrix[4] = -(	(workplateHeight / 1) / (lineTr2Br.length()));
		REXOS_DEBUG_STREAM("lineTl2Tr.length() " << lineTl2Tr.length());
		REXOS_DEBUG_STREAM("lineTr2Br.length() " << lineTr2Br.length());
		REXOS_DEBUG_STREAM("scaleMatrix " << scaleMatrix);
		
		return scaleMatrix;
	}

	void PartLocatorNode::run() {
		REXOS_INFO("waiting for camera/qr_codes");
		ros::Subscriber sub = rexos_module::AbstractModule::nodeHandle.subscribe("camera/qr_codes", 10, &PartLocatorNode::qrCodeCallback, this);
		ros::spin();
	}
	bool PartLocatorNode::transitionInitialize() {
		REXOS_INFO("Initialize transition called");
		return true;
	}

	bool PartLocatorNode::transitionDeinitialize() {
		REXOS_INFO("Deinitialize transition called");
		ros::shutdown();
		return true;
	}


	bool PartLocatorNode::transitionSetup(){
		REXOS_INFO("Setup transition called");
		
		// @TODO Select either deltarobot or six_axisrobot (defaulted to sixaxis in constructor)
		rexos_module::SetInstructionGoal* instructionGoal;
		rexos_module::TransitionGoal transitionGoal;
		
		std::vector<rexos_module::RequiredMutation> requiredMutations;
		rexos_module::RequiredMutation requiredMutation;
		requiredMutation.mutation = "move";
		requiredMutation.isOptional = false;
		requiredMutations.push_back(requiredMutation);
		transitionGoal.requiredMutationsRequiredForNextPhase = requiredMutations;
		
		REXOS_INFO("Waiting for mover");
		transitionActionClient.sendGoal(transitionGoal);
		transitionActionClient.waitForResult();
		rexos_module::TransitionResultConstPtr result = transitionActionClient.getResult();
		
		bool foundCandidate = false;
		rexos_datatypes::ModuleIdentifier moverIdentifier;
		for(rexos_module::CandidateModules candidates : result->candidates) {
			if(candidates.mutation == "move") {
				moverIdentifier = rexos_datatypes::ModuleIdentifier(
						candidates.manufacturer[0], candidates.typeNumber[0], candidates.serialNumber[0]);
				foundCandidate = true;
			}
		}
		if(foundCandidate == false) {
			REXOS_ERROR("did not acquire mover");
			return false;
		}
		
		REXOS_INFO_STREAM("Accuired mover " << moverIdentifier);
		rexos_module::ModuleInterface moverInterface(rexos_module::AbstractModule::equipletName, moverIdentifier);
		
		int acceleration = 20;
		double workSpaceHeight = 35;
		Vector3 v;
		
		rexos_datatypes::EquipletStep equipletStep;
		equipletStep.setModuleIdentifier(moverInterface.getModuleIdentifier());
		rexos_datatypes::OriginPlacement originPlacement;
		originPlacement.setOriginPlacementType(rexos_datatypes::OriginPlacement::OriginPlacementType::RELATIVE_TO_EQUIPLET_ORIGIN);
		equipletStep.setOriginPlacement(originPlacement);
		Json::Value instructionData;
		instructionData["move"]["maxAcceleration"] = acceleration;
		
		
		REXOS_INFO("Moving to top left corner");
		v = Vector3(0 - workPlaneWidth / 2, 0 + workPlaneHeight / 2, workSpaceHeight);
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		
		moverInterface.setInstruction("1", equipletStep.toJSON());
		REXOS_INFO("enter diff to X");
		std::cin >> topLeftOffsetX;
		REXOS_INFO("enter diff to Y");
		std::cin >> topLeftOffsetY;
		
		REXOS_INFO("Moving to top right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 + workPlaneHeight / 2, workSpaceHeight);
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		
		moverInterface.setInstruction("2", equipletStep.toJSON());
		REXOS_INFO("enter diff to X");
		std::cin >> topRightOffsetX;
		REXOS_INFO("enter diff to Y");
		std::cin >> topRightOffsetY;
		
		REXOS_INFO("Moving to bottom right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 - workPlaneHeight / 2, workSpaceHeight);
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		
		moverInterface.setInstruction("3", equipletStep.toJSON());
		REXOS_INFO("enter diff to X");
		std::cin >> bottomRightOffsetX;
		REXOS_INFO("enter diff to Y");
		std::cin >> bottomRightOffsetY;
		
		Vector3 A = Vector3(0						, 0						, 1);
		Vector3 B = Vector3(0 + workPlaneWidth	, 0						, 1);
		Vector3 C = Vector3(0 + workPlaneWidth	, 0 - workPlaneHeight	, 1);
		
		Vector3 Aproj = Vector3(A.x + topLeftOffsetX, A.y + topLeftOffsetY, 1);
		Vector3 Bproj = Vector3(B.x + topRightOffsetX, B.y + topRightOffsetY, 1);
		Vector3 Cproj = Vector3(C.x + bottomRightOffsetX, C.y + bottomRightOffsetY, 1);
		
		REXOS_INFO_STREAM("workPlaneWidth " << workPlaneWidth << " workPlaneHeight " << workPlaneHeight);
		REXOS_INFO_STREAM("A " << A << " B " << B << " C " << C);
		REXOS_INFO_STREAM("Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		// fancy calulation here
		// first translate so that A' matches A
		REXOS_INFO_STREAM("x " << -topLeftOffsetX << " y " << -topLeftOffsetY);
		
		Matrix3 postCorrectionTranslationMatrix = Matrix3();
		postCorrectionTranslationMatrix[2] = -topLeftOffsetX;
		postCorrectionTranslationMatrix[5] = -topLeftOffsetY;
		
		Aproj = postCorrectionTranslationMatrix * Aproj;
		Bproj = postCorrectionTranslationMatrix * Bproj;
		Cproj = postCorrectionTranslationMatrix * Cproj;
		
		REXOS_INFO_STREAM(postCorrectionTranslationMatrix);
		REXOS_INFO_STREAM("translated: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		// second rotate so that normalize(A'B') matches normalize(AB)
		// calulate the angle between A' and B'
		double angleAprojBproj = std::atan2(Bproj.y - Aproj.y, Bproj.x - Aproj.x);
		// calulate the angle between A and B
		double angleAB = std::atan2(B.y - A.y, B.x - A.x); // should be 0
		double rotationAngle = angleAB - angleAprojBproj;
		REXOS_INFO_STREAM("angleAprojBproj " << angleAprojBproj << " angleAB " << angleAB << " rotationAngle " << rotationAngle);
		
		Matrix3 postCorrectionRotationMatrix = Matrix3();
		postCorrectionRotationMatrix[0] = cos(rotationAngle);
		postCorrectionRotationMatrix[1] = -sin(rotationAngle);
		postCorrectionRotationMatrix[3] = sin(rotationAngle);
		postCorrectionRotationMatrix[4] = cos(rotationAngle);
		
		Aproj = postCorrectionRotationMatrix * Aproj;
		Bproj = postCorrectionRotationMatrix * Bproj;
		Cproj = postCorrectionRotationMatrix * Cproj;
		
		REXOS_INFO_STREAM(postCorrectionRotationMatrix);
		REXOS_INFO_STREAM("rotated: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		// third shear so that normalize(B'C') mathces normalize(BC)
		// calulate the angle between A' and B'
		double angleBprojCproj = std::atan2(Cproj.y - Bproj.y, Cproj.x - Bproj.x);
		// calulate the angle between A and B
		double angleBC = std::atan2(C.y - B.y, C.x - B.x); // should be -1/2*Pi
		double shearAngle = angleBprojCproj - angleBC;
		double shearFactor = tan(shearAngle);
		ROS_INFO_STREAM("angleBprojCproj " << angleBprojCproj << " angleBC " << angleBC << 
				" shearAngle " << shearAngle << " shearFactor " << shearFactor);
		
		Matrix3 postCorrectionShearMatrix = Matrix3(
				1, shearFactor, 0,
				0, 1, 0,
				0, 0, 1);
		
		
		Aproj = postCorrectionShearMatrix * Aproj;
		Bproj = postCorrectionShearMatrix * Bproj;
		Cproj = postCorrectionShearMatrix * Cproj;
		
		REXOS_INFO_STREAM(postCorrectionShearMatrix);
		REXOS_INFO_STREAM("sheared: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		// fourth scale so that A'B' matches AB and B'C' matches BC
		Matrix3 postCorrectionScaleMatrix = Matrix3();
		double scaleX = (B - A).length() / (Bproj - Aproj).length();
		double scaleY = (C - B).length() / (Cproj - Bproj).length();
		postCorrectionScaleMatrix[0] = scaleX;
		postCorrectionScaleMatrix[4] = scaleY;
		REXOS_INFO_STREAM("scaleX: " << scaleX << " scaleY " << scaleY);
		
		Aproj = postCorrectionScaleMatrix * Aproj;
		Bproj = postCorrectionScaleMatrix * Bproj;
		Cproj = postCorrectionScaleMatrix * Cproj;
		
		REXOS_INFO_STREAM(postCorrectionScaleMatrix);
		REXOS_INFO_STREAM("scale: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		Matrix3 translateToA = Matrix3(1, 0, workPlaneWidth / 2, 0, 1, -workPlaneHeight / 2, 0, 0, 1);
		Matrix3 translateFromA = Matrix3(1, 0, -workPlaneWidth / 2, 0, 1, workPlaneHeight / 2, 0, 0, 1);
		
		REXOS_INFO_STREAM(translateToA);
		REXOS_INFO_STREAM(translateFromA);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		postCorrectionTotalMatrix = translateFromA * postCorrectionScaleMatrix * 
				postCorrectionShearMatrix * postCorrectionRotationMatrix * 
				postCorrectionTranslationMatrix * translateToA;
		
		REXOS_INFO_STREAM(postCorrectionTotalMatrix);

		REXOS_INFO_STREAM("result: A " << A << "	->	 " << postCorrectionTotalMatrix * A);
		REXOS_INFO_STREAM("A " << A);
		REXOS_INFO_STREAM("A " << translateToA * A);
		REXOS_INFO_STREAM("A " << postCorrectionTranslationMatrix * translateToA * A);
		REXOS_INFO_STREAM("A " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
		REXOS_INFO_STREAM("A " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
		REXOS_INFO_STREAM("A " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
		REXOS_INFO_STREAM("A " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
		REXOS_INFO_STREAM("result: B " << B << "	->	 " << postCorrectionTotalMatrix * B);
		REXOS_INFO_STREAM("B " << B);
		REXOS_INFO_STREAM("B " << translateToA * B);
		REXOS_INFO_STREAM("B " << postCorrectionTranslationMatrix * translateToA * B);
		REXOS_INFO_STREAM("B " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
		REXOS_INFO_STREAM("B " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
		REXOS_INFO_STREAM("B " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
		REXOS_INFO_STREAM("B " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
		REXOS_INFO_STREAM("result: C " << C << "	->	 " << postCorrectionTotalMatrix * C);
		REXOS_INFO_STREAM("C " << C);
		REXOS_INFO_STREAM("C " << translateToA * C);
		REXOS_INFO_STREAM("C " << postCorrectionTranslationMatrix * translateToA * C);
		REXOS_INFO_STREAM("C " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
		REXOS_INFO_STREAM("C " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
		REXOS_INFO_STREAM("C " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
		REXOS_INFO_STREAM("C " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		
		v = Vector3(0 + workPlaneWidth / 2, 0 - workPlaneHeight / 2, 1);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		REXOS_INFO_STREAM("v " << v);
		REXOS_INFO_STREAM("v " << translateToA * v);
		REXOS_INFO_STREAM("v " << postCorrectionTranslationMatrix * translateToA * v);
		REXOS_INFO_STREAM("v " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
		REXOS_INFO_STREAM("v " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
		REXOS_INFO_STREAM("v " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
		REXOS_INFO_STREAM("v " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
		
		
		// @TODO this might be removed?
		
		REXOS_INFO("Moving to top left corner");
		v = Vector3(0 - workPlaneWidth / 2, 0 + workPlaneHeight / 2, 1);
		v = postCorrectionTotalMatrix * v;
		v.z = workSpaceHeight;
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		
		moverInterface.setInstruction("4", equipletStep.toJSON());
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
		
		REXOS_INFO("Moving to top right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 + workPlaneHeight / 2, 1);
		v = postCorrectionTotalMatrix * v;
		v.z = workSpaceHeight;
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		
		moverInterface.setInstruction("5", equipletStep.toJSON());
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
		
		REXOS_INFO("Moving to bottom right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 - workPlaneHeight / 2, 1);
		v = postCorrectionTotalMatrix * v;
		v.z = workSpaceHeight;
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
			
		moverInterface.setInstruction("6", equipletStep.toJSON());
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
		
		REXOS_INFO("Moving origin");
		v = Vector3(0, 0, 1);
		v = postCorrectionTotalMatrix * v;
		v.z = workSpaceHeight;
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		
		moverInterface.setInstruction("7", equipletStep.toJSON());
		
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
		
		return true;
	}
	bool PartLocatorNode::transitionShutdown() {
		REXOS_INFO("Shutdown transition called");
		return true;
	}
	bool PartLocatorNode::transitionStart() {
		REXOS_INFO("Start transition called");
		return true;
	}
	bool PartLocatorNode::transitionStop() {
		REXOS_INFO("Stop transition called");
		return true;
	}

}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "part_locator_node");
	
	std::string equipletName = argv[1];
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[2], argv[3], argv[4]);
	
	REXOS_INFO("Creating PartLocatorNode");
	part_locator_node::PartLocatorNode node(equipletName, moduleIdentifier);
	node.run();
	
	return 0;
}
