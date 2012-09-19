FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/deltaRobotNode/msg"
  "src/deltaRobotNode/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/deltaRobotNode/MoveToRelativePoint.h"
  "srv_gen/cpp/include/deltaRobotNode/MoveToRelativePoints.h"
  "srv_gen/cpp/include/deltaRobotNode/MoveToPoint.h"
  "srv_gen/cpp/include/deltaRobotNode/MoveToPoints.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
