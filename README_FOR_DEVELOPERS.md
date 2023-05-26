# CommonRoad-Autoware Motion Planning Interface

## Table of Contents

- [CommonRoad-Autoware Motion Planning Interface](#commonroad-autoware-motion-planning-interface)
  - [Table of Contents](#table-of-contents)
  - [Setup - Rocker](#setup---rocker)
    - [1. Install Docker and Rocker](#1-install-docker-and-rocker)
    - [2. Download the the autoware repository](#2-download-the-the-autoware-repository)
    - [3. Build the Docker image and run the container](#3-build-the-docker-image-and-run-the-container)
    - [4. Download the codebase](#4-download-the-codebase)
    - [5. Build the workspace](#5-build-the-workspace)
  - [Setup - VNC](#setup---vnc)
    - [1. Install Docker and Rocker](#1-install-docker-and-rocker-1)
    - [2. Download the autoware repository](#2-download-the-autoware-repository)
    - [3. Build the Docker image](#3-build-the-docker-image)
    - [4. Setup VNC](#4-setup-vnc)
    - [4b. Setup Mesa Drivers (only for MacOS)](#4b-setup-mesa-drivers-only-for-macos)
    - [5. Run the Docker container](#5-run-the-docker-container)
    - [6. Test your setup](#6-test-your-setup)
  - [Development](#development)
    - [Pre-Commit Hooks](#pre-commit-hooks)
    - [Pushing to this repository](#pushing-to-this-repository)
    - [Push a new docker image](#push-a-new-docker-image)
    - [Modifications to Autoware](#modifications-to-autoware)

## Setup - Rocker

### 1. Install Docker and Rocker

Follow the steps listed under `Install Dependencies` [here](https://wiki.tum.de/pages/viewpage.action?pageId=1208844439).

### 2. Download the the autoware repository

Clone AV2.0 autoware repository and checkout branch _[integrate_cr2autoware_interface](https://gitlab.lrz.de/av2.0/autoware/-/tree/integrate_cr2autoware_interface)_.

```shell
git clone https://gitlab.lrz.de/av2.0/autoware.git
cd autoware
git checkout integrate_cr2autoware_interface
```

You should be prompted to input your LRZ credentials.

### 3. Build the Docker image and run the container

Follow steps 2 and 3 of the Rocker Workflow [here](https://wiki.tum.de/pages/viewpage.action?pageId=1208844439). **Note: If you are running on an arm64 machine, add the flage `--platform linux/amd64` to the Docker build command**

**@TODO: Test this:**

The python dependencies for CommonRoad are installed for user `tum` in the Dockerfile.
Therefore you need to launch the container as user `tum`:

```shell
rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --user-override-name tum --user-preserve-home --volume {PATH_TO_AUTOWARE}:/home/tum/autoware -- autoware_image
```

<details>
<summary>Explanation</summary>

`-e LIBGL_ALWAYS_SOFTWARE=1` is required for OpenGL.

`--x11` enables display forwarding.

`--user` adds the current (host) user to the container.

`--user-override-name tum` overrides the username to `tum`.

`--user-preserve-home` preserves the home directory of the user (as to not overwrite anything).

`--volume {PATH_TO_AUTOWARE}/autoware:/home/tum/autoware` mounts the autoware repository to the home directory of the user.

`--` separates the rocker arguments from the docker arguments.

</details>

Once you have launched the container, you can test whether display forwarding works with `xeyes` and `glxgears`. These can be installed by running:

```shell
sudo apt-get install mesa-utils
```

### 4. Download the codebase

Follow step 4 of the Rocker Workflow [here](https://wiki.tum.de/pages/viewpage.action?pageId=1208844439).

Verify that the pulled autoware.universe and tum_launch repos are on the correct branches (see above)

### 5. Build the workspace

Follow the rest of the steps of the Rocker Workflow [here](https://wiki.tum.de/pages/viewpage.action?pageId=1208844439) to build the workspace and launch ROS!

## Setup - VNC

<details>
  <summary>Background</summary>

Rocker is a tool that is built on top of Docker. When you run a standard Docker container with Rocker, it automatically sets up some useful features for you, such as:

- Mounting your home directory into the container
- Mounting your X11 socket into the container
- GPU support

  Depending on your platform, Rocker is **not** strictly necessary. If you are running on an Ubuntu x64 machine with [X Server](https://www.x.org/archive/X11R7.7/doc/man/man1/Xserver.1.xhtml) preinstalled, [Rocker Setup](#setup---rocker) is the easiest option. If you are running on a different platform, you may need to setup VNC manually.

  You can read more about the Autoware Docker images and Rocker [here](https://gitlab.lrz.de/av2.0/autoware/-/blob/0683a499d191a21c9142959641394da73313c918/docker/README.md)

</details>

### 1. Install Docker and Rocker

Follow the instructions at <https://docs.docker.com/get-docker/> to install Docker for your platform.

### 2. Download the autoware repository

Clone AV2.0 autoware repository and checkout branch _[integrate_cr2autoware_interface](https://gitlab.lrz.de/av2.0/autoware/-/tree/integrate_cr2autoware_interface)_.

```shell
git clone https://gitlab.lrz.de/av2.0/autoware.git
cd autoware
git checkout integrate_cr2autoware_interface
```

You should be prompted to input your LRZ credentials.

### 3. Build the Docker image

```shell
docker build -t autoware_image --platform linux/amd64 . -f autoware/docker/tum_docker/Dockerfile
```

 <details>
 <summary>Explanation:</summary>
This will build the docker image from scratch.

- `-t autoware_image` - name of the image
- `-f autoware/docker/tum_docker/Dockerfile` - path to the Dockerfile
- `.` - path to the build context (the directory from which the build is run). Make sure you run the command from within the `autoware` directory.
- `--platform` - specify the platform to build for (e.g., `linux/amd64` or `linux/arm64`). Note that CommonRoad **only supports `linux/amd64` at the moment**.

</details>

<details>
<summary>Tip for MacOS users with Apple Silicone (M1, M2 etc.)</summary>

While it is **highly adviced against**, if you want to run ROS on a M1 Mac, make sure to force the Docker engine to use Rosetta emulation. At the time of writing, this feature is still in development. To activate, open your Docker Dekstop Settings, go to `Features in development` and check `Use Rosetta for x86/amd64 emulation on Apple Silicon`.

Also, do not use Parallels or other virtualization software that runs Ubuntu aarch64. This will force the Docker client _inside_ the virtual machine to do the x86/amd64 emulation, which will be very slow.

</details>

### 4. Setup VNC

The Docker image you have previously built now contains your development environment with all necessary dependencies.
However, you still need to setup a VNC server inside the container to be able to view the GUIs of the tools you are using.

There are many options for setting up a VNC server inside a Docker container. Following is a simple example (adjust to your liking).

<details>
<summary>Example Dockerfile</summary>

```dockerfile
FROM autoware_image
# VNC Setup
RUN apt-get update -y && \
    apt-get install -y \
    novnc \
    supervisor \
    openbox \
    tigervnc-standalone-server

COPY supervisord.conf .
COPY entrypoint.sh .
ENV DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768

CMD supervisord -c supervisord.conf > /dev/null && bash
EXPOSE 8080
EXPOSE 5900
```

</details>

This Dockerfile copies the `supervisord.conf` into the container. Make sure to add it to your workspace root:

<details>
<summary>supervisord.conf</summary>

```conf
[supervisord]
nodaemon=false
logfile=/dev/fd/1
logfile_maxbytes=0

[program:websockify]
command=websockify --web /usr/share/novnc 8080 localhost:5900
autorestart=true

[program:openbox]
priority=1
command=/usr/bin/openbox
environment=DISPLAY=:0
autorestart=true
stdout_logfile=/dev/fd/1
stdout_logfile_maxbytes=0
redirect_stderr=true

[program:x11]
priority=0
command=/usr/bin/Xtigervnc +iglx -rfbport 5900 -SecurityTypes None,TLSNone -AlwaysShared -AcceptKeyEvents -AcceptPointerEvents -AcceptSetDesktopSize -SendCutText -AcceptCutText :0
autorestart=true
stdout_logfile=/dev/fd/1
stdout_logfile_maxbytes=0
redirect_stderr=true
```

</details>

### 4b. Setup Mesa Drivers (only for MacOS)

Apple does not support current OpenGL versions. It is therefore necessary to install Mesa drivers to enable OpenGL support in the Docker container.

Once again, there are different options for installing Mesa drivers. The following is an example of how you can install Mesa drivers in your Dockerfile.

<details>
<summary>Example Dockerfile</summary>

```dockerfile
# Install Mesa
RUN sudo apt-get update -y && \
    sudo apt-get install -y \
    x11vnc \
    xterm \
    xvfb \
    mesa-utils \
    x11-apps \
    software-properties-common
RUN sudo add-apt-repository ppa:kisak/kisak-mesa -y
RUN sudo apt-get update --fix-missing -y && \
    sudo apt-get full-upgrade -y && \
    sudo apt-get clean
```

</details>
Building this file might take a while. Be patient.

### 5. Run the Docker container

Now that you have augmented the Dockerfile with a VNC Server and Mesa drivers, you can finally build and run image.

The easiest way is to run the image using VSCode Devcontainers as described [here](https://gitlab.lrz.de/av2.0/autoware/-/blob/0683a499d191a21c9142959641394da73313c918/docker/README.md#tips)

Start by installing the [VSCode Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and create a new `.devcontainer` folder in your workspace.

This is a sample `devcontainer.json` file that you can use to run the Docker container.

<details>
<summary>devcontainer.json</summary>

```json
{
  "name": "Autoware with VNC",
  "build": {
    // Set the context of the Docker build
    "context": "..",
    // Path to the Dockerfile to use to build the container.
    "dockerfile": "../VNC.Dockerfile"
  },
  "containerEnv": {
    "DISPLAY": ":0", // Set the display to the host display
    "LIBGL_ALWAYS_SOFTWARE": "1" // Force OpenGL to use software rendering
  },
  // Mount the workspace folder into the container at a reproducible location
  "mounts": [
    "source=${localWorkspaceFolder}/autoware,target=/home/tum/av20/autoware,type=bind,consistency=cached"
  ],
  "forwardPorts": [
    8080, // VNC webserver
    5900 // VNC port
  ],
  // Launch the VNC server on container startup
  "postStartCommand": "supervisord -c $HOME/supervisord.conf > /dev/null",
  // Important! Make sure to always use user `tum`
  "remoteUser": "tum"
}
```

</details>

### 6. Test your setup

Once you have launched your container, make sure that display forwarding works.

```shell
xeyes
```

You should see two eyes in a separate window. If you use a VNC Webserver navigate to <http://localhost:8080/vnc.html>.

To check, whether OpenGL works, run

```shell
glxgears
```

You should see a window with rotating gears.

If all of this works, you are ready build, run and develop. Continue with building the workspace and running Autoware as described in step 6 [here](https://wiki.tum.de/pages/viewpage.action?pageId=1208844439).

## Development

### Pre-Commit Hooks

To ensure code quality and style consistency, we use pre-commit hooks.
Pre-commit should be preinstalled in your development environment.
To install the hooks, run:

```shell
cd autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car
pre-commit install
```

To ease with local development, we recommend installing `black`, `isort` and `markdownlint` in your local environment.

### Pushing to this repository

If you have set up your development environment as described above, this repo has been cloned as a submodule of the Autoware repo. To push your changes to this repo, you first need to add this repository as a remote.
Run the following command:

```shell
cd autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car
git remote set-url origin git@gitlab.lrz.de:cps/dfg-car.git
```

### Push a new docker image

To update the docker image in the GitLab container registry, run the following commands (change the GitLab address if you are working with a different repository, e.g., the AV2.0 repo):

1. Make the desired changes to your code
2. Build the docker image, e.g., via: `docker build -t autoware_image . -f autoware/docker/tum_docker/Dockerfile`
3. Rename/Tag the image : `docker tag autoware_image gitlab.lrz.de:5005/cps/dfg-car:latest`
4. Push the image to the container registry: `docker push gitlab.lrz.de:5005/cps/dfg-car:latest`

### Modifications to Autoware

See the [TUM-Launch Wiki page](https://gitlab.lrz.de/cps/dfg-car/-/wikis/TUM-Launch) for the list of changes made to
autoware, autoware.universe and tum.launch for the integration of the interface and how to replicate them.

_Note: When updating the autoware version, make sure that the documented changes aren't overwritten._
