FROM osrf/ros:galactic-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  git \
  python3-pip \
  wget \
  tar \
  python3-pykdl \
  sqlite3 \
  python3-tk \
  libxtst6 \
  python3-pil \
  python3-pil.imagetk

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

RUN cd && git clone https://github.com/swri-robotics/swri_console.git \
  && cd swri_console \
  && git checkout ros2-devel \
  && . /opt/ros/galactic/setup.sh \
  && colcon build

RUN cd && git clone https://github.com/tier4/autoware_auto_msgs.git \
  && cd autoware_auto_msgs \
  && . /opt/ros/galactic/setup.sh \
  && colcon build

# Clean up unnecessary files
RUN cd && rm -rf \
  proj-9.0.0.tar.gz

COPY ./docker-entrypoint.sh /
RUN chmod a+x /docker-entrypoint.sh

ENTRYPOINT [ "/docker-entrypoint.sh" , "bash"]