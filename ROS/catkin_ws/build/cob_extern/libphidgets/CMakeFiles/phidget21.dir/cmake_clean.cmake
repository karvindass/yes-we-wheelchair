FILE(REMOVE_RECURSE
  "/home/wes/catkin_ws/devel/lib/libphidget21.pdb"
  "/home/wes/catkin_ws/devel/lib/libphidget21.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/phidget21.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
