ARG BASE_IMAGE
FROM ${BASE_IMAGE}

#
# root
#

RUN python -m pip install \
  lxml
RUN pip install dearpygui==2.0.0

# holiday ros2
ENV HOLIDAY_ROS_ROOT=/holiday/ros2

ARG USER_NAME
RUN mkdir -p ${HOLIDAY_ROS_ROOT}/src \
  && chown -R ${USER_NAME}:${USER_NAME} ${HOLIDAY_ROS_ROOT}

#
# user
#

USER ${USER_NAME}
