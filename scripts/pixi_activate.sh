# Sourced by pixi on activation: overlay this checkout's colcon install space
# on the pixi ROS environment once `pixi run build` has created it.
if [ -f "$PIXI_PROJECT_ROOT/install/local_setup.sh" ]; then
  . "$PIXI_PROJECT_ROOT/install/local_setup.sh"
fi
