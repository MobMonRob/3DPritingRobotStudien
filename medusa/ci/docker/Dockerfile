# ----------------------------------------------------------------------------
# Medusa CI Dockerfile
#
# This Dockerfile sets up a build environment for the Medusa project.
# It installs all required dependencies, configures build tools, and prepares
# the system for building and testing C++ code with modern toolchains.
# ----------------------------------------------------------------------------
FROM ubuntu:22.04

# Set non-interactive mode for apt and terminal color support
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# Configure CMake to use parallel builds and ccache for faster compilation
ENV CMAKE_BUILD_PARALLEL_LEVEL=8
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache
ENV CMAKE_C_COMPILER_LAUNCHER=ccache

# ----------------------------------------------------------------------------
# Install system dependencies and development tools
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    pkg-config \
    python3-pip \
    ccache \
    mold \
    clang-format \
    clang-tidy \
    doxygen \
    graphviz \
    libgl1-mesa-dev \
    xorg-dev \
    libwayland-dev \
    libxkbcommon-dev \
    xvfb \
    libassimp-dev \
    libglfw3-dev \
    libglm-dev \
    libgtest-dev \
    nlohmann-json3-dev \
    libspdlog-dev \
    libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Configure ccache (compiler cache)
# ----------------------------------------------------------------------------
RUN mkdir -p /root/.ccache
RUN ccache -M 5G           # Set maximum cache size to 5GB
RUN ccache -z              # Zero statistics

# ----------------------------------------------------------------------------
# Build and install Google Test (gtest)
# ----------------------------------------------------------------------------
RUN cd /usr/src/gtest && \
    cmake . && \
    make && \
    cp lib/*.a /usr/lib

# -----------------------------------------------------------------------------
# Pre-compile GLAD (OpenGL 4.6 Core profile loader)
# -----------------------------------------------------------------------------
RUN pip install glad && \
    glad --generator=c --api="gl=4.6" --profile="core" --out-path=/usr/local && \
    gcc -c /usr/local/src/glad.c -I/usr/local/include -fPIC -o /usr/local/src/glad.o && \
    ar rcs /usr/local/lib/libglad.a /usr/local/src/glad.o && \
    rm /usr/local/src/glad.c /usr/local/src/glad.o

# ----------------------------------------------------------------------------
# Build and install custom CMake utilities for Medusa
# ----------------------------------------------------------------------------
COPY ci/cmake /tmp/ci_cmake
WORKDIR /tmp/ci_cmake/build

RUN cmake -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=mold" \
    -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=mold" \
    .. && \
    cmake --build . --target install && \
    rm -rf /tmp/ci_cmake

# ----------------------------------------------------------------------------
# Set working directory for application code
# ----------------------------------------------------------------------------
WORKDIR /app