FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/obstacle_navigation/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/obstacle_navigation/msg/__init__.py"
  "../src/obstacle_navigation/msg/_Blobs.py"
  "../src/obstacle_navigation/msg/_Blob.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
