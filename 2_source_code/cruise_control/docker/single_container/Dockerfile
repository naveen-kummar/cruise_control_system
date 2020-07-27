FROM ros:dashing

# install ros build tools
RUN apt-get update && apt-get install -y \
      python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*


# install ssh-agent
RUN apt-get update && apt-get -y install ssh
 

# clone privvate repo for cruise control using ssh
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
COPY github_key .
RUN  eval $(ssh-agent) && \
    ssh-add github_key && \
    ssh-keyscan -H gitlab-public.kpit.com >> /etc/ssh/ssh_config && \
     git -C src clone \
      -b cruise_v1_review_comments_2 \
      git@gitlab.kpit.com:naveenk8/cruisecontrolsystem.git

# install ros package dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y \
      --from-paths \
        src/cruise_control \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*


# build ros package source
RUN . /opt/ros/dashing/setup.sh && \
    colcon build \
      --packages-select \
        cruise_control \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release

# copy ros package install via multi-stage
ENV ROS_WS /opt/ros_ws
COPY --from=0  $ROS_WS/install $ROS_WS/install

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run ros package launch file
CMD ["ros2", "launch", "cruise_control", "cruise_control_node.launch.py"]