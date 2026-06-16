FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgmp-dev \
    libboost-dev \
    libboost-serialization-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/d4
COPY . .

# Build the base d4 library (needed by the server target)
RUN mkdir -p build && cd build && cmake .. && make -j$(nproc)

# Build the d4_kcmcp_server target
RUN mkdir -p server/build && cd server/build && cmake .. && make d4_kcmcp_server -j$(nproc)

EXPOSE 50055

ENTRYPOINT ["/opt/d4/server/build/d4_kcmcp_server", "0.0.0.0:50055"]
