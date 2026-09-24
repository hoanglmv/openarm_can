# ==============================================================================
# OpenArm CAN & Simulation Web Dashboard Dockerfile
# ==============================================================================
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

# Install core build tools, C++ libraries, SocketCAN tools, and Python
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    libcli11-dev \
    git \
    can-utils \
    iproute2 \
    net-tools \
    kmod \
    python3 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir --break-system-packages \
    scikit-build-core \
    nanobind \
    websockets

WORKDIR /workspace

# Copy repository files
COPY . /workspace

# Build and install OpenArm C++ library & openarm-can-cli
RUN cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build \
    && cmake --install build \
    && ldconfig

# Install OpenArm Python bindings
RUN pip install --no-cache-dir --break-system-packages -e ./python || true

# Web Dashboard HTTP & WebSocket ports
EXPOSE 8888 8889

# Default entrypoint runs OpenArm Web & Telemetry server
CMD ["python3", "sim/server.py"]
