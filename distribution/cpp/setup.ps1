$navground_root = $PSScriptRoot
$Env:NAVGROUND_DLL_PATH = $navground_root + "\bin"
$Env:CMAKE_PREFIX_PATH = $navground_root + ";" + $Env:CMAKE_PREFIX_PATH
$Env:Path = $navground_root + "\bin;" + $navground_root + "\lib;" + $Env:Path
$Env:NAVGROUND_PLUGINS_PREFIX = $navground_root + ";" + $Env:NAVGROUND_PLUGINS_PREFIX