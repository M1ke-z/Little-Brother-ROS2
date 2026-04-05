docker run --name little_brother -it --rm --device /dev/i2c-1:/dev/i2c-1 --device /dev/input:/dev/input  --network host -v ~/ros2_ws:/ws ros2-little_brother #ros:jazzy-ros-base bash
# docker exec little_brother cd ws
# docker exec little_brother source install/setup.bash
# docker exec -it little_brother /bin/bash 