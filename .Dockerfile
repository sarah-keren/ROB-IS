# ROB-is docker image
FROM kclplanning/rosplan

SHELL ["/bin/bash", "-c"]
WORKDIR /root/ws


COPY . ./src/rob_is
# Autoclone
#RUN git clone https://github.com/sarah-keren/ROB-IS.git src/rob_is 

# Further dependencies
RUN source devel/setup.bash &&\
    rosdep update &&\
    rosdep install --from-paths src/ --ignore-src -q -r -y

# Build workspace
RUN catkin build --summarize --no-status