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
    rosdep install --from-paths src/rob_is src/rosplan_demos --ignore-src -q -r -y &&\
    apt install ros-melodic-rviz-visual-tools ssh-client -y -qq

RUN  echo "IdentityFile ~/.ssh/rob_is-docker" >> /etc/ssh/ssh_config

# REMOVE ME ONCE MERGED IN MASTER
RUN cd /root/ws/src/rosplan_demos && git checkout -b sarah-keren-master master && git pull https://github.com/sarah-keren/rosplan_demos.git master

# Build workspace
RUN catkin build --summarize --no-status

# Important to run this from the run_docker.sh script!!!!
ENTRYPOINT ["bash", "/root/ws/rob_is-results/run_experiment.sh"]