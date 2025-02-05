root=$(dirname $(readlink -f ${(%):-%x}))
export CMAKE_PREFIX_PATH=$root:$CMAKE_PREFIX_PATH
export PATH=$root/bin:$PATH
export DYLD_LIBRARY_PATH=$root/lib:$DYLD_LIBRARY_PATH
export NAVGROUND_PLUGINS_PREFIX=$root:$NAVGROUND_PLUGINS_PREFIX