Run all of the following from ros2_ws when needed

# For finding the missing my_python_pkg
source install/setup.bash

# build code after adding python changes
colcon build

# Pull the docker image using:

docker pull osrf/ros:humble-desktop

# Run the docker image for creating a container:
docker run -it osrf/ros:humble-desktop

# Check the running container ID
docker ps

# attach a terminal to the running container
docker exec -it <container_id> bash
