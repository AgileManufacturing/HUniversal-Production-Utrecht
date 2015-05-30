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

create table Part(
  name char(200) NOT NULL,
  positionX double NOT NULL,
  positionY double NOT NULL,
  gazeboModel int NOT NULL,
  attachedToLeft int NOT NULL,
  attachedToRight int NOT NULL,
  primary key (name),
  foreign key (gazeboModel) references GazeboModel(id) ON DELETE NO ACTION ON UPDATE NO ACTION
);

create table Equiplet(
  name char(200) NOT NULL,
  positionX double NOT NULL,
  positionY double NOT NULL,
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
  halSoftware int NOT NULL,
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
  equiplet char(200) NULL,
  moduleProperties text NULL,
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

