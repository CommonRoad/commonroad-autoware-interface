# syntax=docker/dockerfile:1
FROM osrf/ros:galactic-desktop

SHELL ["/bin/bash", "-c"]

# RUN useradd -ms /bin/bash drivingsim

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  git \
  python3-pip \
  wget \
  tar \
  python3-pykdl \
  sqlite3 \
  python3-tk

RUN pip --no-input install 'python-dateutil>=2.8.2' \
  commonroad-drivability-checker \
  ipywidgets \
  pyproj \
  tqdm

RUN cd && wget https://download.osgeo.org/proj/proj-9.0.0.tar.gz

RUN cd && tar -xvf proj-9.0.0.tar.gz \
  && cd proj-9.0.0 && mkdir build && cd build \
  && cmake .. \
  && cmake --build . \
  && cmake --build . --target install \
  && ldconfig \
  && ln -s /usr/local/lib /usr/lib

RUN cd && git clone https://gitlab.lrz.de/tum-cps/commonroad-scenario-designer.git \
  && cd commonroad-scenario-designer \
  && pip install -e .

RUN cd && git clone https://github.com/tier4/autoware_auto_msgs.git

# Clean up unnecessary files
RUN cd && rm -rf \
  proj-9.0.0.tar.gz

#CMD source ~/workspace/dfg-car/install/setup.bash && source /opt/ros/foxy/setup.bash