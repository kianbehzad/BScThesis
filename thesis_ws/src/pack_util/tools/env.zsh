#!/bin/zsh

# Save the directory this script is located in
SCRIPTS_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
# Find the parent directory that corresponds to the Parsian root
PARSIAN_ROOT="$SCRIPTS_DIR"
while [[ ! -d "$PARSIAN_ROOT/parsian_ws" ]]; do
	PARSIAN_ROOT=$(dirname "$PARSIAN_ROOT")
	if [[ "$PARSIAN_ROOT" == "/" ]]; then
		echo "Warning: Could not find ROS catkin workspace!"
		return
	fi
done
PARSIAN_ROOT="$PARSIAN_ROOT/parsian_ws"

# source project setup.zsh
 if [[ ! -f "$PARSIAN_ROOT/install/setup.zsh" ]]; then
    echo "WARNING: couldn't find setup.zsh, please build the project and then try again"
  else
    source "$PARSIAN_ROOT/install/setup.zsh"
  fi


  function parsian() {
	TEMP_DIR=$(pwd)
	cd $PARSIAN_ROOT
	case "$1" in
		build)
			colcon build
			;;
		rebuild)
			rm -rf build
			rm -rf install
			rm -rf log
			colcon build
			;;
		*)
			cd $TEMP_DIR
			echo "Unrecognised parsian command!"
			echo "Try: parsian help"
			;;
	esac
	cd $TEMP_DIR
}







