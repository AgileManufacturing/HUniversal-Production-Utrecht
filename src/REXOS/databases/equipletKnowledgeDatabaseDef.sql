-- LOCAL ---
drop table if exists ServiceDependencySet;
drop table if exists Service;
drop table if exists Equiplet;
drop table if exists ModuleCalibrationGroup;
drop table if exists ModuleCalibration;
drop table if exists Module;
drop table if exists ModuleType;

create table ModuleType(
	manufacturer char(200) NOT NULL,
	typeNumber char(200) NOT NULL,
	moduleTypeProperties text NOT NULL,
	rosSoftwareBuildNumber int NOT NULL,
	rosSoftware longblob NOT NULL,
	masSoftwareBuildNumber int NOT NULL,
	masSoftware longblob NOT NULL,
	primary key (manufacturer, typeNumber)
);

create table Module(
	manufacturer char(200) NOT NULL,
	typeNumber char(200) NOT NULL,
	serialNumber char(200) NOT NULL,
	mountPointX int NOT NULL,
	mountPointY int NOT NULL,
	attachedToManufacturer char(200) NULL,
	attachedToTypeNumber char(200) NULL,
	attachedToSerialNumber char(200) NULL,
	moduleProperties text NOT NULL,
	primary key (manufacturer, typeNumber, serialNumber),
	foreign key (manufacturer, typeNumber) references ModuleType(manufacturer, typeNumber) ON DELETE CASCADE ON UPDATE NO ACTION
);

create table ModuleCalibration(
	id int NOT NULL AUTO_INCREMENT,
	properties text NOT NULL,
	primary key (id)
);

create table ModuleCalibrationGroup(
	ModuleCalibration int NOT NULL,
	manufacturer char(200) NOT NULL,
	typeNumber char(200) NOT NULL,
	serialNumber char(200) NOT NULL,
	primary key (ModuleCalibration, manufacturer, typeNumber, serialNumber),
	foreign key (manufacturer, typeNumber, serialNumber) references Module(manufacturer, typeNumber, serialNumber) ON DELETE CASCADE ON UPDATE NO ACTION,
	foreign key (ModuleCalibration) references ModuleCalibration(id) ON DELETE CASCADE ON UPDATE NO ACTION
);

create table Equiplet(
	name char(200) NOT NULL,
	mountPointsX int NOT NULL,
	mountPointsY int NOT NULL,
	mountPointDistanceX double NOT NULL,
	mountPointDistanceY double NOT NULL,
	primary key (name)
);

create table Service(
	serviceType char(200) NOT NULL,
	buildNumber int NOT NULL,
	data longblob NOT NULL,
	primary key (serviceType)
);

create table ServiceDependencySet(
	serviceType char(200) NOT NULL,
	setNumber int NOT NULL,
	manufacturer char(200) NOT NULL,
	typeNumber char(200) NOT NULL,
	primary key (serviceType, setNumber),
	foreign key (serviceType) references Service(serviceType) ON DELETE CASCADE ON UPDATE NO ACTION,
	foreign key (manufacturer, typeNumber) references ModuleType(manufacturer, typeNumber) ON DELETE CASCADE ON UPDATE NO ACTION
);

