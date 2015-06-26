drop trigger if exists equiplet_insert;
drop trigger if exists module_insert;

drop table if exists Part;
drop table if exists PartType;
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
drop table if exists GazeboLink;
drop table if exists GazeboJoint;
drop table if exists GazeboCollision;
drop table if exists GazeboModel;
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
create table GazeboModel(
  id int NOT NULL AUTO_INCREMENT,
  buildNumber int NOT NULL,
  zipFile longblob NOT NULL,
  sdfFilename char(200) NOT NULL,
  parentLink char(200) NOT NULL,
  childLink char(200) NOT NULL,
  childLinkOffsetX double NOT NULL,
  childLinkOffsetY double NOT NULL,
  childLinkOffsetZ double NOT NULL,
  primary key (id)
);

create table GazeboCollision(
  gazeboModel int NOT NULL AUTO_INCREMENT,
  linkName char(200) NOT NULL,
  collisionName char(200) NOT NULL,
  maxForce double NOT NULL,
  maxTorque double NOT NULL,
  mayHaveContactWithChildModules bool NOT NULL,
  primary key (gazeboModel, linkName, collisionName),
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table GazeboJoint(
  gazeboModel int NOT NULL AUTO_INCREMENT,
  jointName char(200) NOT NULL,
  maxErrorPose double NOT NULL,
  primary key (gazeboModel, jointName),
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table GazeboLink(
  gazeboModel int NOT NULL AUTO_INCREMENT,
  linkName char(200) NOT NULL,
  maxAcceleration double NOT NULL,
  primary key (gazeboModel, linkName),
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table Equiplet(
  name char(200) NOT NULL,
  mountPointsX int NOT NULL,
  mountPointsY int NOT NULL,
  mountPointDistanceX double NOT NULL,
  mountPointDistanceY double NOT NULL,
  rosSoftware int NOT NULL,
  masSoftware int NOT NULL,
  gazeboModel int NOT NULL,
  primary key (name),
  foreign key (rosSoftware) references RosSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION,
  foreign key (masSoftware) references JavaSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION,
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table ModuleType(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  moduleTypeProperties text NOT NULL,
  rosSoftware int NULL,
  halSoftware int NULL,
  gazeboModel int NOT NULL,
  primary key (manufacturer, typeNumber),
  foreign key (rosSoftware) references RosSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION,
  foreign key (halSoftware) references JavaSoftware(id) ON DELETE NO ACTION ON UPDATE NO ACTION,
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
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

create table PartType(
  typeNumber char(200) NOT NULL,
  partTypeProperties text NOT NULL,
  gazeboModel int NOT NULL,
  primary key (typeNumber),
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);
create table Part(
  partName char(200) NOT NULL,
  partType char(200) NOT NULL,
  attachedToLeft int NOT NULL,
  attachedToRight int NOT NULL,
  positionX double NOT NULL,
  positionY double NOT NULL,
  positionZ double NOT NULL,
  rotationX double NOT NULL,
  rotationY double NOT NULL,
  rotationZ double NOT NULL,
  partProperties text NOT NULL,
  qrCode longblob NULL,
  primary key (partName),
  foreign key (partType) references PartType(typeNumber) ON DELETE NO ACTION ON UPDATE NO ACTION
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
