insert into JavaSoftware
(buildNumber, jarFile, className)
values(1, "", "equiplet");

insert into RosSoftware
(buildNumber, zipFile, command)
values(1, LOAD_FILE("/home/t/git/HUniversal-Production-Utrecht/build.xml"), "rosrun equiplet_node equiplet_node {equipletName}");

insert into GazeboModel
(buildNumber, zipFile, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ)
values(1, "", "model.sdf", "baseLink", "otherLink", 0.0, 0.0, 0.0);

insert into GazeboModel
(buildNumber, zipFile, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ)
values(1, "", "model.sdf", "baseLink", "otherLink", 0.0, 0.0, 0.0);

insert into Equiplet
(name, mountPointsX, mountPointsY, mountPointDistanceX, mountPointDistanceY, rosSoftware, masSoftware, gazeboModel)
values("EQ2", 10, 18, 50, 50, 1, 1, 1);
