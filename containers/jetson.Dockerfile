# On the Jetson, this pulls the correct arm64 variant automatically
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

RUN apt-get update && apt-get install -y --no-install-recommends \
    iputils-ping \
    net-tools \
    iproute2 \
    locales \
 && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
