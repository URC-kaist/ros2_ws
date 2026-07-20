# syntax=docker/dockerfile:1.7

# Node's Bullseye build uses an older glibc than Ubuntu 22.04, so its binaries
# can safely be copied into the ROS Humble/Jammy development image.
FROM node:22-bullseye-slim AS node_runtime

FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=mr2
ARG USER_UID=1000
ARG USER_GID=1000

COPY --from=node_runtime /usr/local/ /usr/local/

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        ca-certificates \
        can-utils \
        ccache \
        cmake \
        curl \
        git \
        git-lfs \
        gstreamer1.0-libav \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-tools \
        iproute2 \
        iputils-ping \
        jq \
        less \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev \
        nano \
        nlohmann-json3-dev \
        pkg-config \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ripgrep \
        socat \
        sudo \
        udev \
    && rm -rf /var/lib/apt/lists/*

RUN if ! test -f /etc/ros/rosdep/sources.list.d/20-default.list; then rosdep init; fi

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd \
        --create-home \
        --gid "${USER_GID}" \
        --shell /bin/bash \
        --uid "${USER_UID}" \
        "${USERNAME}" \
    && printf '%s ALL=(root) NOPASSWD:ALL\n' "${USERNAME}" \
        > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

COPY docker/dev/entrypoint.bash /usr/local/bin/mr2-dev-entrypoint
RUN chmod 0755 /usr/local/bin/mr2-dev-entrypoint

ENV ROS_DISTRO=humble

USER ${USERNAME}
WORKDIR /workspace/mr2-stack

ENTRYPOINT ["/usr/local/bin/mr2-dev-entrypoint"]
CMD ["sleep", "infinity"]
