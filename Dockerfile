FROM ubuntu:24.04 AS ps_base
ARG USERNAME=planning
ARG SEMVER_TAG

# Set version
ENV SEMVER_TAG=$SEMVER_TAG



ARG DEBIAN_FRONTEND=noninteractive

# create data directory
RUN mkdir /data /logs

FROM ubuntu:24.04 AS ps_grpcurl
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates wget \
  && update-ca-certificates \
  && rm -rf /var/lib/apt/lists/*
# Setup grpcurl-based healthcheck dependencies used by docker-compose environments
RUN wget https://github.com/fullstorydev/grpcurl/releases/download/v1.9.3/grpcurl_1.9.3_linux_amd64.deb \
  && dpkg -i grpcurl_1.9.3_linux_amd64.deb \
  && install -m 0755 "$(command -v grpcurl)" /usr/local/bin/grpcurl \
  && rm grpcurl_1.9.3_linux_amd64.deb \
  && mkdir -p /healthcheck/proto/grpc/health/v1/ \
  && wget https://raw.githubusercontent.com/grpc/grpc/master/src/proto/grpc/health/v1/health.proto -O /healthcheck/proto/grpc/health/v1/health.proto

FROM ps_base AS ps_prod
USER root
# Install gosu
RUN apt update && apt install -y \
  gosu=1.17-1 \
  && rm -rf /var/lib/apt/lists/*

# Setup healthcheck
COPY --from=ps_grpcurl /usr/local/bin/grpcurl /usr/local/bin/grpcurl
COPY --from=ps_grpcurl /healthcheck /healthcheck

# Copy and install package
COPY ./dist ./dist
# Copy data
COPY ./data /data
# Install planning service package and OMPL
RUN dpkg -i dist/planning_service.deb \
  && cp -r dist/ompl/* /usr/local \
  && rm -rf dist \
  && /opt/drake/share/drake/setup/install_prereqs -y
# Point linker to Drake libs
ENV LD_LIBRARY_PATH=/opt/drake/lib
COPY ./docker_entrypoint.sh /docker_entrypoint.sh
# Create the user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN userdel -r ubuntu
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid  $USER_UID --gid $USER_GID -m -d /root $USERNAME
RUN chown ${USER_UID}:${USER_GID} /root /tmp /data logs
ENTRYPOINT [ "/docker_entrypoint.sh" ]


FROM ps_base AS ps_build
# set directory
ARG APP_DIR="/root/app"
WORKDIR ${APP_DIR}
USER root
# Build dependencies
RUN apt update && apt install -y --no-install-recommends \
  apt-transport-https \
  bzip2 \
  ca-certificates \
  curl \
  doxygen \
  g++ \
  gdb \
  git \
  gnupg \
  gpg \
  lcov \
  libc-ares-dev \
  libre2-dev \
  libglew-dev \
  lsb-release \
  ninja-build \
  pkg-config \
  software-properties-common \
  unzip \
  vim \
  wget

# Setup healthcheck dependencies
COPY --from=ps_grpcurl /usr/local/bin/grpcurl /usr/local/bin/grpcurl
COPY --from=ps_grpcurl /healthcheck /healthcheck

# Bazel (via Bazelisk)
ARG BAZELISK_VERSION=1.20.0
RUN  wget https://github.com/bazelbuild/bazelisk/releases/download/v${BAZELISK_VERSION}/bazelisk-linux-amd64 \
  && mv bazelisk-linux-amd64 /usr/bin/bazel \
  && chmod +x /usr/bin/bazel \
  && bazel --version

# Install drake dependencies
RUN curl -SL https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-noble.tar.gz | tar -xz  \
  && cat drake/share/drake/setup/install_prereqs \
  && drake/share/drake/setup/install_prereqs -y \
  && rm -rf drake

ENV LD_LIBRARY_PATH=/usr/local/lib
# Create the user last for layer cachine
ARG USER_UID=1000
RUN userdel -r ubuntu
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid  $USER_UID --gid $USER_GID -m -d /root $USERNAME
# TODO Don't hardcode /root/app in code so we can move install dir
RUN chown -R ${USER_UID}:${USER_GID} /root /logs

# Set local
RUN apt update && apt install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
