FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/deltaRobotNode/msg"
  "src/deltaRobotNode/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/MoveToRelativePoint.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_MoveToRelativePoint.lisp"
  "srv_gen/lisp/MoveToRelativePoints.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_MoveToRelativePoints.lisp"
  "srv_gen/lisp/MoveToPoint.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_MoveToPoint.lisp"
  "srv_gen/lisp/MoveToPoints.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_MoveToPoints.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
