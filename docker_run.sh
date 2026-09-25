docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/src:/ros2_ws/src \
  -v $(pwd)/results:/ros2_ws/results \
  -v $(pwd)/logs:/ros2_ws/logs \
  ros2-humble-dev
