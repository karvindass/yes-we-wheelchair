FILE(REMOVE_RECURSE
  "/home/wes/catkin_ws/devel/lib/libntcan.pdb"
  "/home/wes/catkin_ws/devel/lib/libntcan.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ntcan.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
