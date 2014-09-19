drop trigger if exists equiplet_insert;
drop trigger if exists module_insert;

drop table if exists ServiceType_CapabilityType;
drop table if exists ServiceType;
drop table if exists CapabilityTypeRequiredMutation;
drop table if exists CapabilityType;
drop table if exists ModuleCalibrationModuleSet;
drop table if exists ModuleCalibration;
drop table if exists SupportedCalibrationMutation;
drop table if exists RequiredCalibrationMutation;
drop table if exists SupportedMutation;
drop table if exists Module;
drop table if exists ModuleType;
drop table if exists Equiplet;
drop table if exists RosSoftware;
drop table if exists JavaSoftware;

create table JavaSoftware(
  id int NOT NULL AUTO_INCREMENT,
  buildNumber int NOT NULL,
  jarFile longblob NOT NULL,
  className char(200),
  primary key (id)
);
create table RosSoftware(
  id int NOT NULL AUTO_INCREMENT,
  buildNumber int NOT NULL,
  zipFile longblob NOT NULL,
  command char(200),
  primary key (id)
);

create table Equiplet(
  name char(200) NOT NULL,
  mountPointsX int NOT NULL,
  mountPointsY int NOT NULL,
  mountPointDistanceX double NOT NULL,
  mountPointDistanceY double NOT NULL,
  rosSoftware int NOT NULL,
  masSoftware int NOT NULL,
  primary key (name),
  foreign key (rosSoftware) references RosSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION,
  foreign key (masSoftware) references JavaSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table ModuleType(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  moduleTypeProperties text NOT NULL,
  rosSoftware int NULL,
  halSoftware int NOT NULL,
  primary key (manufacturer, typeNumber),
  foreign key (rosSoftware) references RosSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION,
  foreign key (halSoftware) references JavaSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table Module(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  serialNumber char(200) NOT NULL,
  equiplet char(200) NOT NULL,
  mountPointX int NULL,
  mountPointY int NULL,
  attachedToLeft int NOT NULL,
  attachedToRight int NOT NULL,
  moduleProperties text NOT NULL,
  primary key (manufacturer, typeNumber, serialNumber),
  foreign key (equiplet) references Equiplet(name) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table SupportedMutation(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  mutation char(200) NOT NULL,
  primary key (manufacturer, typeNumber, mutation),
  foreign key (manufacturer, typeNumber) references ModuleType(manufacturer, typeNumber) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table RequiredCalibrationMutation(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  phase int NOT NULL,
  mutation char(200) NOT NULL,
  isOptional tinyint(1) NOT NULL,
  primary key (manufacturer, typeNumber, phase, mutation),
  foreign key (manufacturer, typeNumber) references ModuleType(manufacturer, typeNumber) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table SupportedCalibrationMutation(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  phase int NOT NULL,
  mutation char(200) NOT NULL,
  primary key (manufacturer, typeNumber, phase, mutation),
  foreign key (manufacturer, typeNumber) references ModuleType(manufacturer, typeNumber) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table ModuleCalibration(
  id int NOT NULL AUTO_INCREMENT,
  date datetime NOT NULL,
  properties text NOT NULL,
  primary key (id, date)
);

create table ModuleCalibrationModuleSet(
  ModuleCalibration int NOT NULL,
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  serialNumber char(200) NOT NULL,
  primary key (ModuleCalibration, manufacturer, typeNumber, serialNumber),
  -- foreign key (manufacturer, typeNumber, serialNumber) references Module(manufacturer, typeNumber, serialNumber) ON DELETE CASCADE ON UPDATE NO ACTION,
  foreign key (ModuleCalibration) references ModuleCalibration(id) ON DELETE CASCADE ON UPDATE NO ACTION
);

create table CapabilityType(
  name char(200) NOT NULL,
  halSoftware int NOT NULL,
  primary key (name),
  foreign key (halSoftware) references JavaSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table CapabilityTypeRequiredMutation(
  treeNumber int NOT NULL,
  capabilityType char(200) NOT NULL,
  mutation char(200) NOT NULL,
  primary key (treeNumber, capabilityType, mutation),
  foreign key (capabilityType) references CapabilityType(name) ON DELETE CASCADE ON UPDATE NO ACTION
);

create table ServiceType(
  name char(200) NOT NULL,
  primary key (name)
);

create table ServiceType_CapabilityType(
  serviceType char(200) NOT NULL,
  capabilityType char(200) NOT NULL,
  primary key (serviceType, capabilityType),
  foreign key (serviceType) references ServiceType(name) ON DELETE CASCADE ON UPDATE NO ACTION,
  foreign key (capabilityType) references CapabilityType(name) ON DELETE CASCADE ON UPDATE NO ACTION
);

DELIMITER $$
create trigger equiplet_insert before insert on Equiplet
FOR EACH ROW
BEGIN
  IF NEW.mountPointDistanceX <= 0 OR NEW.mountPointDistanceY <= 0 THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'equipletMountPointDistance: distance <= 0';
  END IF;
END$$

DELIMITER ;

DELIMITER $$
create trigger module_insert before insert on Module
FOR EACH ROW
BEGIN
  IF NEW.mountPointX >= (
    SELECT mountPointsX
    FROM Equiplet
    WHERE name = NEW.equiplet
  ) OR NEW.mountPointY >= (
    SELECT mountPointsY
    FROM Equiplet
    WHERE name = NEW.equiplet
  ) THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'moduleMountPoint: mountPointX <= mountPointsX || mountPointY <= mountPointsY';
  ELSEIF NEW.mountPointX IS NOT NULL AND NEW.mountPointY IS NOT NULL AND
    (
		SELECT count(*) 
		FROM Module 
		WHERE attachedToLeft < NEW.attachedToLeft AND attachedToRight > NEW.attachedToRight
	) != 0 THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'moduleAttachedToModule: attached to module and mountplate at the same time';
  ELSEIF (NEW.mountPointX IS NULL OR NEW.mountPointY IS NULL) AND 
    (
		SELECT count(*) 
		FROM Module 
		WHERE attachedToLeft < NEW.attachedToLeft AND attachedToRight > NEW.attachedToRight
	) = 0 THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'moduleAttachedToModule: module not attached to mountplate and has no parent module';
  END IF;
END$$
DELIMITER ;

insert into JavaSoftware(
	id,
	buildNumber,
	jarFile,
	className
) values
(1,0, "", "dummy");

insert into RosSoftware(
	id,
	buildNumber,
	zipFile,
	command
) values
(1,0, "", "dummy");

insert into Equiplet
(name, mountPointsX, mountPointsY, mountPointDistanceX, mountPointDistanceY, rosSoftware, masSoftware)
values("EQ1", 10, 10, 50, 50, 1, 1);

insert into ModuleType(
	manufacturer, 
	typeNumber,
	moduleTypeProperties,
	rosSoftware,
	halSoftware
) values 
("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "", 1, 1),
("The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "", 1, 1),
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
}", 1, 1),
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
}", 1, 1), 
("HU", "work_plane_type_A", "{
	\"midPointX\" : 175.0,
	\"midPointY\" : -200.0,
	\"midPointZ\" : 33.33,
	\"topLeftValue\" : \"_WP_TL\",
	\"topRightValue\" : \"_WP_TR\",
	\"bottomRightValue\" : \"_WP_BR\",
	\"workPlaneWidth\" : 80.0,
	\"workPlaneHeight\" : 80.0
}", 1, 1), 
("HU", "gripper_type_A", "{
	\"modbusAddress\" : 8001,
	\"modbusDevicePin\" : 0,
	\"gripperEnabledMaxSeconds\" : 60,
	\"gripperEnabledWarningSeconds\" : 50,
	\"gripperEnabledCooldownSeconds\" : 180,
	\"watchdogInterval\" : 100
}", 1, 1);

insert into Module(
	equiplet,
	manufacturer,
	typeNumber,
	serialNumber,
	mountPointX,
	mountPointY,
	attachedToLeft,
	attachedToRight,
	moduleProperties
) values 
("EQ1", "The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035", 3, 2, 1, 4, ""),
("EQ1", "The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "1", null, null, 2, 3, ""),
("EQ1", "HU", "delta_robot_type_A", "1", 3, 2, 5, 8, "{
	\"modbusIp\" : \"192.168.0.22\",
	\"modbusPort\" : 502
}"),
("EQ1","HU", "six_axis_type_A", "1", 3, 0, 11, 12, "{
	\"modbusIp\" : \"192.168.0.32\",
	\"modbusPort\" : 502
}"),
("EQ1", "HU", "work_plane_type_A", "1", 1, 2, 9, 10, ""),
("EQ1", "HU", "gripper_type_A", "1", null, null, 6, 7, "{
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

insert into ModuleCalibrationModuleSet (ModuleCalibration, manufacturer, typeNumber, serialNumber) values
(1, "The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "1"), 
(1, "The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035");

insert into JavaSoftware
(buildNumber, jarFile, className)
values(1, "", "equiplet");

insert into RosSoftware
(buildNumber, zipFile, command)
values(1, "", "rosrun equiplet_node equiplet_node {equipletName}");

insert into Equiplet
(name, mountPointsX, mountPointsY, mountPointDistanceX, mountPointDistanceY, rosSoftware, masSoftware)
values("EQ3", 10, 18, 50, 50, 1, 1);
