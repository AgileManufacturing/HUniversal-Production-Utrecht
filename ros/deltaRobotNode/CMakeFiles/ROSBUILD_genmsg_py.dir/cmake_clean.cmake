FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/deltaRobotNode/msg"
  "src/deltaRobotNode/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/deltaRobotNode/msg/__init__.py"
  "src/deltaRobotNode/msg/_Motion.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
