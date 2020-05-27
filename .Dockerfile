# ROB-is docker image
FROM kclplanning/rosplan:demos_latest

SHELL ["/bin/bash", "-c"]
WORKDIR /root/ws


COPY . ./src/rob_is
# Autoclone
#RUN git clone https://github.com/sarah-keren/ROB-IS.git src/rob_is 

# Further dependencies
RUN source devel/setup.bash &&\
    rosdep update &&\
    rosdep install --from-paths src/rob_is --ignore-src -q -r -y &&\
    sudo apt install ros-melodic-rviz-visual-tools

# Build workspace
RUN catkin build --summarize --no-status