FROM ros:noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV VIDEO_DEVICE=/dev/video4
COPY . /ros1_jaka_calib_ws
# 相机参数文件
COPY ./head_camera.yaml /root/.ros/camera_info/head_camera.yaml
WORKDIR /ros1_jaka_calib_ws
RUN bash -c \ 
"apt-get update && \
source /opt/ros/noetic/setup.bash && \
rosdep install --from-paths src --ignore-src -r -y && \
apt-get install ros-noetic-moveit-visual-tools python3-pip -y && \
pip install Pillow redis==5.0.1"
RUN bash -c \
"source /opt/ros/noetic/setup.bash && \
catkin_make"
CMD [ "./start.bash" ]
