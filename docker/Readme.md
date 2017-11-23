# Docker Support

#### Pre-requisite

- [What are Containers?](https://aws.amazon.com/containers)
- [What is Docker?](https://en.wikipedia.org/wiki/Docker_(software))

## Description

Docker support is added to ORB-SLAM2 using which user can perform following operations

- Build ORB-SLAM2 from docker
- Run ORB-SLAM2 applications from docker
- Create docker image

### Build ORB-SLAM2 from docker
We can build ORB-SLAM2 using docker, advantages of this method is user need not have to install any dependences in host computer. Docker base image with installed dependences is used to compile ORB-SLAM2

```
cd ORB-SLAM2
./docker/build-from-docker.sh
```

Output build artifacts are stored under `ORB-SLAM2/products`

### Run ORB-SLAM2 applications from docker
Similar to building ORB-SLAM2 using a docker image, we can also run the built ORB-SLAM2 binaries in the same docker image

```
cd ORB-SLAM2
./docker/run-from-docker.sh "<application-binary> <parameter>"
```
Note: Enclosing `"<application-binary> <parameter>"` within in double quotes is mandatory

`run-from-docker.sh` provides way to run any ORB-SLAM2's application binary. `run-garching-video-from-docker.sh` uses `run-from-docker.sh` to run a specific example of ORB-SLAM2.

```
cd ORB-SLAM2
./docker/run-garching-video-from-docker.sh <path-to-video>
```

Note that `<path-to-video>` must be a path to a video file inside of the `ORB_SLAM2` folder. Host's current directory i.e. `ORB-SLAM2` is shared under `/root/orb-slam2` path, inside the container. Hence, it is required to place the video file under this shared directory `ORB-SLAM2`.

### Create Docker image
Optionally user can create new docker image locally, which can be used to build and run ORB-SLAM2.

```
cd ORB-SLAM2
./docker/create-image.sh
```
Above script creates a docker image, provisioned with required dependences for ORB-SLAM2

## Details

- `create-image.sh` creates a docker image named `shanmukhananda/orb-slam2:latest` in local computer
- Creating docker image can be time consuming hence, this image has been uploaded to [Docker Hub](https://hub.docker.com/r/shanmukhananda/orb-slam2), a free cloud storage for docker images.
- `build-from-docker.sh` and `run-from-docker.sh` first tries to find the image `shanmukhananda/orb-slam2:latest` in local computer, if image is not present then the image is downloaded from Docker Hub.
- Running ORB-SLAM2's application involves OpenGL rendering. Hence, it is required for docker containers running ORB-SLAM2 appliction to access host's display interfaces like, display server port, IPC, rendering interfaces. Easy way to provide access to these resources is to execute `run-from-docker.sh` as `root` user.

#### Reference
- [Using GUI's with Docker](http://wiki.ros.org/docker/Tutorials/GUI)
- [Using Hardware Acceleration with Docker](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)
