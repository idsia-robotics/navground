root=$(dirname $(readlink -f ${(%):-%x}))
export CMAKE_PREFIX_PATH=$root:$CMAKE_PREFIX_PATH
export PATH=$root/bin:$PATH
export LD_LIBRARY_PATH=$root/lib:$LD_LIBRARY_PATH
export NAVGROUND_PLUGINS_PREFIX=$root:$NAVGROUND_PLUGINS_PREFIX