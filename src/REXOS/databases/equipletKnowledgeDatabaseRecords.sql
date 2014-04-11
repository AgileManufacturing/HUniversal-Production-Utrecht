insert into ModuleType(
	manufacturer, 
	typeNumber,
	moduleTypeProperties,
	rosSoftwareBuildNumber,
	rosSoftware,
	masSoftwareBuildNumber,
	masSoftware
) values 
("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "", 1, "", 1, ""),
("The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "", 1, "", 1, ""),
("HU", "delta_robot_type_A", "{
	\"midPointX\" : 75.0,
	\"midPointY\" : -200.0,
	\"midPointZ\" : -35.3,
	\"deltaRobotMeasures\" : {
		\"baseRadius\" : 101.3,
		\"hipLength\" : 100.0,
		\"effectorRadius\" : 46.19,
		\"ankleLength\" : 250.0,
		\"hipAnleMaxAngleDegrees\" : 22.0,
		\"motorFromZeroToTopAngleDegrees\" : 20.0,
		\"boundaryBoxMinX\" : -200.0,
		\"boundaryBoxMaxX\" : 200.0,
		\"boundaryBoxMinY\" : -200.0,
		\"boundaryBoxMaxY\" : 200.0,
		\"boundaryBoxMinZ\" : -330.0,
		\"boundaryBoxMaxZ\" : -180.0
	}, 
	\"calibrationBigStepFactor\" : 20,
	\"stepperMotorProperties\" : {
		\"motorMinAngleDegrees\" : -18.0,
		\"motorMaxAngleDegrees\" : 80.0,
		\"microStepAngleDegrees\" : 0.036,
		\"minAccelerationDegrees\" : 36,
		\"maxAccelerationDegrees\" : 36000,
		\"minSpeedDegrees\" : 0.036,
		\"maxSpeedDegrees\" : 18000
	}
}", 1, "", 1, ""),
("HU", "six_axis_type_A", "{
	\"midPointX\" : 75.0,
	\"midPointY\" : -200.0,
	\"midPointZ\" : -35.3,
	\"stewartGoughMeasures\" : {
		\"baseRadius\" : 101.3,
		\"hipLength\" : 100.0,
		\"effectorRadius\" : 46.19,
		\"ankleLength\" : 250.0,
		\"hipAnleMaxAngleDegrees\" : 22.0,
		\"motorFromZeroToTopAngleDegrees\" : 20.0,
		\"boundaryBoxMinX\" : -200.0,
		\"boundaryBoxMaxX\" : 200.0,
		\"boundaryBoxMinY\" : -200.0,
		\"boundaryBoxMaxY\" : 200.0,
		\"boundaryBoxMinZ\" : -330.0,
		\"boundaryBoxMaxZ\" : -180.0
	}, 
	\"calibrationBigStepFactor\" : 20,
	\"stepperMotorProperties\" : {
		\"motorMinAngleDegrees\" : -18.0,
		\"motorMaxAngleDegrees\" : 80.0,
		\"microStepAngleDegrees\" : 0.036,
		\"minAccelerationDegrees\" : 36,
		\"maxAccelerationDegrees\" : 36000,
		\"minSpeedDegrees\" : 0.036,
		\"maxSpeedDegrees\" : 18000
	}
}", 1, "", 1, ""),
("HU", "work_plane_type_A", "{
	\"midPointX\" : 175.0,
	\"midPointY\" : -200.0,
	\"midPointZ\" : 33.33,
	\"topLeftValue\" : \"_WP_TL\",
	\"topRightValue\" : \"_WP_TR\",
	\"bottomRightValue\" : \"_WP_BR\",
	\"workPlaneWidth\" : 80.0,
	\"workPlaneHeight\" : 80.0
}", 1, "", 1, ""), 
("HU", "gripper_type_A", "{
	\"modbusAddress\" : 8001,
	\"modbusDevicePin\" : 0,
	\"gripperEnabledMaxSeconds\" : 60,
	\"gripperEnabledWarningSeconds\" : 50,
	\"gripperEnabledCooldownSeconds\" : 180,
	\"watchdogInterval\" : 100
}", 1, "", 1, "");

insert into Module(
	manufacturer,
	typeNumber,
	serialNumber,
	mountPointX,
	mountPointY,
	attachedToManufacturer,
	attachedToTypeNumber,
	attachedToSerialNumber,
	moduleProperties
) values 
("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035", 3, 2, null, null, null, ""),
("The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "1", 6, 4, "The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035", ""),
("HU", "delta_robot_type_A", "1", 3, 2, null, null, null, "{
	\"modbusIp\" : \"192.168.0.22\",
	\"modbusPort\" : 502
}"),
("HU", "six_axis_type_A", "1", 3, 2, null, null, null, "{
	\"modbusIp\" : \"192.168.0.32\",
	\"modbusPort\" : 502
}"),
("HU", "work_plane_type_A", "1", 1, 10, null, null, null, ""),
("HU", "gripper_type_A", "1", 1, 10, null, null, null, "{
	\"modbusIp\" : \"192.168.0.22\",
	\"modbusPort\" : 502
}");







insert into ModuleCalibration (id, properties) values
(1, "{
	\"distCoeffs\" : {
		\"\" : -0.584008,
		\"\" : -4.858694,
		\"\" : -0.009096,
		\"\" : -0.016636,
		\"\" : 40.603377
	},
	\"cameraMatrix\" : {
		\"\" : 846.623497,
		\"\" : 0,
		\"\" : 358.965449,
		\"\" : 0,
		\"\" : 851.128986,
		\"\" : 222.244692,
		\"\" : 0,
		\"\" : 0,
		\"\" : 1
	}
}");

insert into ModuleCalibrationGroup (ModuleCalibration, manufacturer, typeNumber, serialNumber) values
(1, "The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "1"), 
(1, "The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035");

