FROM ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential cmake git pkg-config \
    libeigen3-dev libflann-dev libqt5core5a libqt5gui5 libqt5widgets5 qtbase5-dev \
    libopencv-dev libpcl-dev \
    libzmq3-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Build RTAB-Map (Standalone lib, no ROS)
WORKDIR /opt
RUN git clone --depth 1 https://github.com/introlab/rtabmap.git
WORKDIR /opt/rtabmap/build
RUN cmake .. -DWITH_PCL=ON -DWITH_QT=OFF -DWITH_OPENMP=ON -DCMAKE_BUILD_TYPE=Release \
 && cmake --build . -j2 \
 && cmake --install . --prefix /usr/local   

# Bridge
WORKDIR /app
COPY CMakeLists.txt /app/
COPY src /app/src
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2

# ZMQ endpoints (SUB to Windows host, PUB for your RTSM)
ENV SUB=tcp://host.docker.internal:5555
ENV PUB=tcp://*:6000
# Set to "true" to include full RGB-D data in kf_packet (default: false for thumbnail only)
ENV FULL_RGBD=false
ENTRYPOINT ["/app/build/rtabmap_zmq_bridge"]
