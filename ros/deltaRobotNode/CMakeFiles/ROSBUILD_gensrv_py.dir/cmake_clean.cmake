FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/deltaRobotNode/msg"
  "src/deltaRobotNode/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/deltaRobotNode/srv/__init__.py"
  "src/deltaRobotNode/srv/_MoveToRelativePoint.py"
  "src/deltaRobotNode/srv/_MoveToRelativePoints.py"
  "src/deltaRobotNode/srv/_MoveToPoint.py"
  "src/deltaRobotNode/srv/_MoveToPoints.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
