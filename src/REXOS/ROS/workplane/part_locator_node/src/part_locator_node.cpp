#include <part_locator_node/part_locator_node.h>

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>
#include <algorithm>
#include <vector>

#include <environment_cache/EnvironmentCache.h>
#include <rexos_utilities/Utilities.h>
#include <rexos_datatypes/HardwareStep.h>
#include <rexos_knowledge_database/KnowledgeDatabaseException.h>

namespace part_locator_node {
	const Vector2 PartLocatorNode::EXPECTED_DIRECTION = Vector2(-1, 0);
	const Vector2 PartLocatorNode::EXPECTED_ITEM_DIRECTION = Vector2(-1, 0);
	const int PartLocatorNode::minCornerSamples = 11;
	const int PartLocatorNode::minItemSamples = 11;
	const int PartLocatorNode::workSpaceHeight = 35;

	PartLocatorNode::PartLocatorNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow):
			rexos_module::Module(equipletName, moduleIdentifier, isSimulated, isShadow),
			environmentCacheClient(rexos_module::AbstractModule::nodeHandle.serviceClient<environment_cache::setData>(equipletName + "/setData")),
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
				
				Vector3 newCoor = totalMatrix * oldCoor;
				newCoor = postCorrectionTotalMatrix * newCoor;

				/*if(message.qrCodes[i].value == "GC4x4MB_1") {
					REXOS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\toldCoor \t" << oldCoor << "newCoor \t" << newCoor);
				}*/

				points[j] = Vector2(newCoor.x, newCoor.y);
			}
			Vector2 lineA2B = points[1] - points[0];
			Vector2 lineB2C = points[2] - points[1];
			
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
		
		double expectedItemAngle = acos(EXPECTED_ITEM_DIRECTION.x);
		if(EXPECTED_ITEM_DIRECTION.y < 0) expectedItemAngle = 0 - expectedItemAngle;
		double actualItemAngle = acos(actualItemDirection.x);
		if(actualItemDirection.y < 0) actualItemAngle = 0 - actualItemAngle;
		
		double angle = actualItemAngle - expectedItemAngle;
		
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
		totalMatrix = calculateWorkPlaneScaleMatrix() * calculateWorkPlaneRotationMatrix() * calculateWorkPlaneOffsetMatrix();
		//REXOS_INFO_STREAM("totalMatrix " << totalMatrix);
		
	}
	Matrix3 PartLocatorNode::calculateWorkPlaneOffsetMatrix() {
		////////////
		// calulate midpoint
		////////////
		// line between topLeft and topRight
		Vector2 lineTl2Tr;
		lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
		lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
		
		// line between topRight and bottomRight
		Vector2 lineTr2Br;
		lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
		lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
		
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
		
		Matrix3 translationMatrix;
		translationMatrix[2] = -midPoint.x;
		translationMatrix[5] = -midPoint.y;
		
		return translationMatrix;
	}
	Matrix3 PartLocatorNode::calculateWorkPlaneRotationMatrix() {
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
		
		return rotationMatrix;
	}
	Matrix3 PartLocatorNode::calculateWorkPlaneScaleMatrix() {
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
		
		return scaleMatrix;
	}
	
	Matrix3 PartLocatorNode::calculateOffsetMatrix() {
		Matrix3 postCorrectionTranslationMatrix = Matrix3();
		postCorrectionTranslationMatrix[2] = -topLeftOffsetX;
		postCorrectionTranslationMatrix[5] = -topLeftOffsetY;
		
		return postCorrectionTranslationMatrix;
	}
	
	Matrix3 PartLocatorNode::calculateRotationMatrix(double rotationAngle) { 
		Matrix3 postCorrectionRotationMatrix = Matrix3();
		postCorrectionRotationMatrix[0] = cos(rotationAngle);
		postCorrectionRotationMatrix[1] = -sin(rotationAngle);
		postCorrectionRotationMatrix[3] = sin(rotationAngle);
		postCorrectionRotationMatrix[4] = cos(rotationAngle);
		
		return postCorrectionRotationMatrix;
	}
	
	Matrix3 PartLocatorNode::calculateShearMatrix(double shearFactor) {
		Matrix3 postCorrectionShearMatrix = Matrix3(
				1, shearFactor, 0,
				0, 1, 0,
				0, 0, 1);

		return postCorrectionShearMatrix;
	}
	Matrix3 PartLocatorNode::calculateScaleMatrix(double scaleX, double scaleY) {
		Matrix3 postCorrectionScaleMatrix = Matrix3();

		postCorrectionScaleMatrix[0] = scaleX;
		postCorrectionScaleMatrix[4] = scaleY;
		
		return postCorrectionScaleMatrix;
	}

	void PartLocatorNode::MoveToPoint(Vector3 v, std::string hardwarestepId, rexos_module::ModuleInterface& moverInterface){
		int acceleration = 20;
		
		rexos_datatypes::HardwareStep equipletStep;
		equipletStep.setModuleIdentifier(moverInterface.getModuleIdentifier());
		rexos_datatypes::OriginPlacement originPlacement;
		originPlacement.setOriginPlacementType(rexos_datatypes::OriginPlacement::OriginPlacementType::RELATIVE_TO_EQUIPLET_ORIGIN);
		equipletStep.setOriginPlacement(originPlacement);
		Json::Value instructionData;
		instructionData["move"]["maxAcceleration"] = acceleration;
		
		v.z = workSpaceHeight;
		v = convertToEquipletCoordinate(v);
		instructionData["move"]["x"] = v.x;
		instructionData["move"]["y"] = v.y;
		instructionData["move"]["z"] = v.z;
		equipletStep.setInstructionData(instructionData);
		moverInterface.executeHardwareStep(equipletStep);
	}
	
	bool PartLocatorNode::mannuallyCalibrate(rexos_datatypes::ModuleIdentifier moverIdentifier){
		
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

		for(rexos_module::CandidateModules candidates : result->candidates) {
			if(candidates.mutation == "move") {
				moverIdentifier = rexos_datatypes::ModuleIdentifier(
						candidates.manufacturer[0], candidates.typeNumber[0], candidates.serialNumber[0]);
				foundCandidate = true;
			}
		}
		if(foundCandidate == false) {
			REXOS_ERROR("Did not acquire mover");
			return false;
		}
		
		REXOS_INFO_STREAM("Accuired mover " << moverIdentifier);
		rexos_module::ModuleInterface moverInterface(rexos_module::AbstractModule::equipletName, moverIdentifier);
		// Go to every corner and ask the userinput for the difference in mm to the corner of the QR code (thus the black corner)
		Vector3 v;
				
		REXOS_INFO("Moving to top left corner");
		v = Vector3(0 - workPlaneWidth / 2, 0 + workPlaneHeight / 2, workSpaceHeight);
		MoveToPoint(v, "1", moverInterface);

		REXOS_INFO("enter diff to X");
		std::cin >> topLeftOffsetX;
		REXOS_INFO("enter diff to Y");
		std::cin >> topLeftOffsetY;
		
		REXOS_INFO("Moving to top right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 + workPlaneHeight / 2, workSpaceHeight);
		MoveToPoint(v, "2", moverInterface);

		REXOS_INFO("enter diff to X");
		std::cin >> topRightOffsetX;
		REXOS_INFO("enter diff to Y");
		std::cin >> topRightOffsetY;
		
		REXOS_INFO("Moving to bottom right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 - workPlaneHeight / 2, workSpaceHeight);
		MoveToPoint(v, "3", moverInterface);
		
		REXOS_INFO("enter diff to X");
		std::cin >> bottomRightOffsetX;
		REXOS_INFO("enter diff to Y");
		std::cin >> bottomRightOffsetY;
///////////////////////////////////////////////////////UP HERE FIRST ITTERATION////////////////////////////////////////////////////////////////////		
		Vector3 A = Vector3(0					, 0						, 1);
		Vector3 B = Vector3(0 + workPlaneWidth	, 0						, 1);
		Vector3 C = Vector3(0 + workPlaneWidth	, 0 - workPlaneHeight	, 1);
		
		Vector3 Aprojected = Vector3(A.x + topLeftOffsetX, A.y + topLeftOffsetY, 1);
		Vector3 Bprojected = Vector3(B.x + topRightOffsetX, B.y + topRightOffsetY, 1);
		Vector3 Cprojected = Vector3(C.x + bottomRightOffsetX, C.y + bottomRightOffsetY, 1);
		
		REXOS_INFO_STREAM("workPlaneWidth " << workPlaneWidth << " workPlaneHeight " << workPlaneHeight);
		REXOS_INFO_STREAM("A " << A << " B " << B << " C " << C);
		REXOS_INFO_STREAM("Aprojected " << Aprojected << " Bprojected " << Bprojected << " Cprojected " << Cprojected);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
//////////////////////////////////////////////////////////////////FROM HERE STARTS MATRIX CALC//////////////////////////////////////////////////////////////////////////////////////////		
		// first translate so that A' matches A
		REXOS_INFO_STREAM("x " << -topLeftOffsetX << " y " << -topLeftOffsetY);
		
		Matrix3 postCorrectionTranslationMatrix = calculateOffsetMatrix();
		
		Aprojected = postCorrectionTranslationMatrix * Aprojected;
		Bprojected = postCorrectionTranslationMatrix * Bprojected;
		Cprojected = postCorrectionTranslationMatrix * Cprojected;
		
		// second rotate so that normalize(A'B') matches normalize(AB)
		// calulate the angle between A' and B'
		double angleAprojectedBprojected = std::atan2(Bprojected.y - Aprojected.y, Bprojected.x - Aprojected.x);
		// calulate the angle between A and B
		double angleAB = std::atan2(B.y - A.y, B.x - A.x);
		double rotationAngle = angleAB - angleAprojectedBprojected;
		
		Matrix3 postCorrectionRotationMatrix = calculateRotationMatrix(rotationAngle);

		Aprojected = postCorrectionRotationMatrix * Aprojected;
		Bprojected = postCorrectionRotationMatrix * Bprojected;
		Cprojected = postCorrectionRotationMatrix * Cprojected;
		
		// third shear so that normalize(B'C') mathces normalize(BC)
		// calulate the angle between A' and B'
		double angleBprojectedCprojected = std::atan2(Cprojected.y - Bprojected.y, Cprojected.x - Bprojected.x);
		// calulate the angle between A and B
		double angleBC = std::atan2(C.y - B.y, C.x - B.x);
		double shearAngle = angleBprojectedCprojected - angleBC;
		double shearFactor = tan(shearAngle);
		
		Matrix3 postCorrectionShearMatrix = calculateShearMatrix(shearFactor);
		
		Aprojected = postCorrectionShearMatrix * Aprojected;
		Bprojected = postCorrectionShearMatrix * Bprojected;
		Cprojected = postCorrectionShearMatrix * Cprojected;
		
		REXOS_INFO_STREAM(postCorrectionShearMatrix);
		REXOS_INFO_STREAM("sheared: Aprojected " << Aprojected << " Bprojected " << Bprojected << " Cprojected " << Cprojected);
		REXOS_WARN_STREAM("--------------------------------------------------------------");
		
		// fourth scale so that A'B' matches AB and B'C' matches BC

		double scaleX = (B - A).length() / (Bprojected - Aprojected).length();
		double scaleY = (C - B).length() / (Cprojected - Bprojected).length();

		Matrix3 postCorrectionScaleMatrix = calculateScaleMatrix(scaleX, scaleY);
				
		Aprojected = postCorrectionScaleMatrix * Aprojected;
		Bprojected = postCorrectionScaleMatrix * Bprojected;
		Cprojected = postCorrectionScaleMatrix * Cprojected;
	
		Matrix3 translateToA = Matrix3(1, 0, workPlaneWidth / 2, 0, 1, -workPlaneHeight / 2, 0, 0, 1);
		Matrix3 translateFromA = Matrix3(1, 0, -workPlaneWidth / 2, 0, 1, workPlaneHeight / 2, 0, 0, 1);
		
		postCorrectionTotalMatrix = translateFromA * postCorrectionScaleMatrix * 
				postCorrectionShearMatrix * postCorrectionRotationMatrix * 
				postCorrectionTranslationMatrix * translateToA;
		
		REXOS_INFO("Moving to top left corner");
		v = Vector3(0 - workPlaneWidth / 2, 0 + workPlaneHeight / 2, 1);
		v = postCorrectionTotalMatrix * v;
		MoveToPoint(v, "4", moverInterface);
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();

		REXOS_INFO("Moving to top right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 + workPlaneHeight / 2, 1);
		v = postCorrectionTotalMatrix * v;
		MoveToPoint(v, "5", moverInterface);
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
	
		REXOS_INFO("Moving to bottom right corner");
		v = Vector3(0 + workPlaneWidth / 2, 0 - workPlaneHeight / 2, 1);
		v = postCorrectionTotalMatrix * v;
		MoveToPoint(v, "6", moverInterface);
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
	
		REXOS_INFO("Moving origin");
		v = Vector3(0, 0, 1);
		v = postCorrectionTotalMatrix * v;
		MoveToPoint(v, "7", moverInterface);
		REXOS_INFO("Press enter when ready");
		cin.ignore();
		cin.get();
		
		
		Json::Value jsonNode;
		std::vector<rexos_datatypes::ModuleIdentifier> modules;
		modules.push_back(moverIdentifier);
		try{
			
			std::string properties = this->getCalibrationDataForModuleAndOtherModules(modules);
			Json::Reader reader;
			reader.parse(properties, jsonNode);
		} catch(rexos_knowledge_database::KnowledgeDatabaseException ex) {
			// there is no calibration data, so we start with a new node (which we created above)
		}
		
		Json::Value topLeftNode;
		topLeftNode["x"] = topLeftOffsetX;
		topLeftNode["y"] = topLeftOffsetY;
		jsonNode["TL"] = topLeftNode;
		
		Json::Value topRightNode;
		topRightNode["x"] = topRightOffsetX;
		topRightNode["y"] = topRightOffsetY;
		jsonNode["TR"] = topRightNode;
		
		Json::Value bottomRightNode;
		bottomRightNode["x"] = bottomRightOffsetX;
		bottomRightNode["y"] = bottomRightOffsetY;
		jsonNode["BR"] = topRightNode;
		
		Json::StyledWriter writer;
		REXOS_INFO_STREAM("JSON=" << writer.write(jsonNode));
		this->setCalibrationDataForModuleAndOtherModules(modules, writer.write(jsonNode));
		return true;
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

	bool PartLocatorNode::transitionSetup() {
		REXOS_INFO("Setup transition called");
		
		rexos_datatypes::ModuleIdentifier moverIdentifier;
		
		std::vector<rexos_datatypes::ModuleIdentifier> modules;
		modules.push_back(this->getModuleIdentifier());
		modules.push_back(moverIdentifier);
		
		try {
			std::string properties = this->getCalibrationDataForModuleAndOtherModules(modules);
			Json::Value jsonNode;
			Json::Reader reader;
			reader.parse(properties, jsonNode);		
			
			if(jsonNode.isMember("TL") == false || jsonNode.isMember("TR") == false || jsonNode.isMember("BR") == false) {
				if(mannuallyCalibrate(moverIdentifier) == false){
					return false;
				}
			}	
			return true;
		} catch(rexos_knowledge_database::KnowledgeDatabaseException ex) {
			if(mannuallyCalibrate(moverIdentifier) == false) return false;
			else return true;
		}
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
	if(argc < 5){
		REXOS_ERROR("Usage: part_locator_node (--isSimulated | --isShadow) equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isSimulated") {
			isSimulated = true;
		} else if (arg == "--isShadow") {
			isShadow = true;
			isSimulated = true;
		}
	}
	
	std::string equipletName = std::string(argv[argc - 4]);
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[argc - 3], argv[argc - 2], argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_" + moduleIdentifier.getManufacturer() + "_" + 
			moduleIdentifier.getTypeNumber() + "_" + moduleIdentifier.getSerialNumber();
	ros::init(argc, argv, nodeName);
	
	REXOS_INFO("Creating PartLocatorNode");
	part_locator_node::PartLocatorNode node(equipletName, moduleIdentifier, isSimulated, isShadow);
	
	ros::spin();
	return 0;
}
