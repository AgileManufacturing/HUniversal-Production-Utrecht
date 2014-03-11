drop trigger if exists equiplet_insert;
drop trigger if exists module_insert;

drop table if exists ServiceType_CapabilityType;
drop table if exists ServiceType;
drop table if exists CapabilityTypeRequiredMutation;
drop table if exists CapabilityType;
drop table if exists ModuleCalibrationModuleSet;
drop table if exists ModuleCalibration;
drop table if exists SupportedMutation;
drop table if exists Module;
drop table if exists ModuleType;
drop table if exists Equiplet;
drop table if exists HalSoftware;

create table HalSoftware(
  id int NOT NULL AUTO_INCREMENT,
  className char(200) NOT NULL,
  jarFile longblob NOT NULL,
  primary key(id)
);

create table Equiplet(
  name char(200) NOT NULL,
  mountPointsX int NOT NULL,
  mountPointsY int NOT NULL,
  mountPointDistanceX double NOT NULL,
  mountPointDistanceY double NOT NULL,
  softwareBuildNumber int NOT NULL,
  rosSoftware longblob NOT NULL,
  HalSoftware_id int NOT NULL,
  primary key (name)
);

create table ModuleType(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  moduleTypeProperties text NOT NULL,
  rosSoftwareBuildNumber int NOT NULL,
  rosSoftware longblob NOT NULL,
  HalSoftware_id int NOT NULL,
  primary key (manufacturer, typeNumber)
);

create table Module(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  serialNumber char(200) NOT NULL,
  equiplet char(200) NULL,
  mountPointX int NOT NULL,
  mountPointY int NOT NULL,
  attachedToLeft int NOT NULL,
  attachedToRight int NOT NULL,
  moduleProperties text NOT NULL,
  primary key (manufacturer, typeNumber, serialNumber),
  foreign key (equiplet) references Equiplet(name) ON DELETE CASCADE ON UPDATE NO ACTION
);

create table SupportedMutation(
  manufacturer char(200) NOT NULL,
  typeNumber char(200) NOT NULL,
  mutation char(200) NOT NULL,
  primary key (manufacturer, typeNumber, mutation),
  foreign key (manufacturer, typeNumber) references ModuleType(manufacturer, typeNumber) ON DELETE CASCADE ON UPDATE NO ACTION
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
  foreign key (manufacturer, typeNumber, serialNumber) references Module(manufacturer, typeNumber, serialNumber) ON DELETE CASCADE ON UPDATE NO ACTION,
  foreign key (ModuleCalibration) references ModuleCalibration(id) ON DELETE CASCADE ON UPDATE NO ACTION
);

create table CapabilityType(
  name char(200) NOT NULL,
  HalSoftware_id int NOT NULL,
  primary key (name)
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
  IF NEW.mountPointX <= (
    SELECT mountPointsX
    FROM Equiplet
    WHERE name = Module.equiplet
  ) OR NEW.mountPointY <= (
    SELECT mountPointsY
    FROM Equiplet
    WHERE name = Module.equiplet
  ) THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'moduleMountPoint: mountPointX <= mountPointsX || mountPointY <= mountPointsY';
  ELSEIF NEW.equiplet IS NOT NULL AND 
    (NEW.mountPointX IS NULL OR NEW.mountPointY IS NULL) THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'moduleAttachedToModule: attached to equiplet and mountplate IS NULL';
  ELSEIF NEW.attachedToLeft + 1 != NEW.attachedToRight AND 
    (NEW.mountPointX IS NOT NULL OR NEW.mountPointY IS NOT NULL OR NEW.equiplet IS NULL) THEN
    SIGNAL SQLSTATE 'Z0001'
      SET MESSAGE_TEXT = 'moduleAttachedToModule: attached to module and mountplate IS NULL or equiplet IS NULL';
  END IF;
END$$
DELIMITER ;