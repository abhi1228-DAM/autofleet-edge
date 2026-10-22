FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install langchain openai pydantic requests

ENV TURTLEBOT3_MODEL=waffle
WORKDIR /ros2_ws
CMD ["bash"]
