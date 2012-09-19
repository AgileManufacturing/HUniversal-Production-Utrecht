FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/deltaRobotNode/msg"
  "src/deltaRobotNode/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/Motion.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Motion.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
