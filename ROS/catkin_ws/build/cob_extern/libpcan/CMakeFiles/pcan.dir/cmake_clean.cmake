FILE(REMOVE_RECURSE
  "/home/wes/catkin_ws/devel/lib/libpcan.pdb"
  "/home/wes/catkin_ws/devel/lib/libpcan.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/pcan.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
