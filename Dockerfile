FROM usdotfhwastol/carma-base:carma-system-4.2.0 AS base_image

FROM base_image AS build

ARG ROS1_PACKAGES=""
ENV ROS1_PACKAGES=${ROS1_PACKAGES}
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

COPY --chown=carma . /home/carma/autoware.ai
RUN /home/carma/autoware.ai/docker/checkout.bash
RUN ./home/carma/autoware.ai/docker/install.sh

FROM base_image

ARG BUILD_DATE="NULL"
ARG VCS_REF="NULL"
ARG VERSION="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="autoware.ai"
LABEL org.label-schema.description="Binary applications and libraries from autoware.ai for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/autoware.ai"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --chown=carma --from=build /opt/autoware.ai/ros/install /opt/autoware.ai/ros/install
COPY --chown=carma --from=build /opt/autoware.ai/ros/install_ros2 /opt/autoware.ai/ros/install_ros2
