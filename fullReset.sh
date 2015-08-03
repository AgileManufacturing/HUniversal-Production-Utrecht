#! /bin/bash

# clean everything up
mysql -u rexos -p equiplet < src/REXOS/databases/knowledgeDatabaseDef.sql ; 
rm -r generatedOutput/ ; 
rm -r /tmp/rexos_model_spawner/ ; 
rm -r /tmp/rexos_node_spawner/ ; 

trap 'echo "ERROR"; trap - SIGQUIT SIGTERM SIGINT; trap - ERR; return;' ERR ; 
trap 'echo "Exiting"; cleanUp; trap - SIGQUIT SIGTERM SIGINT; trap - ERR; return;' SIGQUIT SIGTERM SIGINT ; 

# copy compiled plugins to the worlds
mkdir -p generatedOutput/worlds/ ; 
cp devel/lib/libacceleration_plugin.so generatedOutput/worlds/ ; 
cp devel/lib/libcollision_plugin.so generatedOutput/worlds/ ; 
cp devel/lib/libjoint_plugin.so generatedOutput/worlds/ ; 
# copy compiled plugins to the models
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/HU/delta_robot_type_B/_model/ ; 
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/HU/gripper_type_A/_model/ ; 
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/HU/gripper_type_B/_model/ ; 
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/HU/workplane_type_A/_model/ ; 
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/HU/stewart_gough_type_A/_model/ ; 
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/The_Imaging_Source_Europe_GmbH/cheap_ass_lens/_model/ ; 
cp devel/lib/libattach_plugin.so 		src/REXOS/modules/The_Imaging_Source_Europe_GmbH/DFK_22AUC03/_model/ ; 
cp devel/lib/libmotor_manager_plugin.so 	src/REXOS/modules/HU/delta_robot_type_B/_model/ ; 
cp devel/lib/libmotor_manager_plugin.so 	src/REXOS/modules/HU/stewart_gough_type_A/_model/ ; 
cp devel/lib/libsensor_manager_plugin.so 	src/REXOS/modules/HU/delta_robot_type_B/_model/ ; 
cp devel/lib/libsensor_manager_plugin.so 	src/REXOS/modules/HU/stewart_gough_type_A/_model/ ; 
cp devel/lib/libgripper_plugin.so 		src/REXOS/modules/HU/gripper_type_A/_model/ ; 
cp devel/lib/libgripper_plugin.so 		src/REXOS/modules/HU/gripper_type_B/_model/ ; 
# export rosSoftware, halSoftware, models
ant generate-all ;
# export staticSettings
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.ModuleRecordGenerator ;
# insert parts
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader chessboard_1 chessboard ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_1	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_2	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_3	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_4	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_5	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_6	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_1	redBall		GC4x4MB_1 -16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_2	redBall		GC4x4MB_1 -5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_3	redBall		GC4x4MB_1 5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_4	redBall		GC4x4MB_1 16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_5	redBall		GC4x4MB_1 -16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_6	redBall		GC4x4MB_1 -5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_7	redBall		GC4x4MB_1 5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_8	redBall		GC4x4MB_1 16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_9	redBall		GC4x4MB_1 -16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_10	redBall		GC4x4MB_1 -5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_11	redBall		GC4x4MB_1 5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_12	redBall		GC4x4MB_1 16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_13	redBall		GC4x4MB_1 -16.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_14	redBall		GC4x4MB_1 -5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_15	redBall		GC4x4MB_1 5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_16	redBall		GC4x4MB_1 16.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_1	blueBall	GC4x4MB_2 -16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_2	blueBall	GC4x4MB_2 -5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_3	blueBall	GC4x4MB_2 5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_4	blueBall	GC4x4MB_2 16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_5	blueBall	GC4x4MB_2 -16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_6	blueBall	GC4x4MB_2 -5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_7	blueBall	GC4x4MB_2 5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_8	blueBall	GC4x4MB_2 16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_9	blueBall	GC4x4MB_2 -16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_10	blueBall	GC4x4MB_2 -5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_11	blueBall	GC4x4MB_2 5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_12	blueBall	GC4x4MB_2 16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_13	blueBall	GC4x4MB_2 -16.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_14	blueBall	GC4x4MB_2 -5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_15	blueBall	GC4x4MB_2 5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_16	blueBall	GC4x4MB_2 16.5	-16.5 	10 0 0 0 ;
# insert equiplets
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ2 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ3 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQD ;
# insert modules
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClass --noTranslate EQ0;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassPickAndPlace --noTranslate EQ2;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassStewartGough --noTranslate EQ3;

trap - SIGQUIT SIGTERM SIGINT; 
trap - ERR; 
