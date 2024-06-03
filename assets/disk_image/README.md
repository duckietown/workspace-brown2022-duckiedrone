# Generate a disk image for brown2022

The following tutorial explains how the disk image for `brown2022` is created.

## Base (input) image

The base image for this course is `dt-raspios-bullseye-lite-v3.0.8-arm64v8.img`
and can be downloaded from the official duckietown repository on the DCSS.
The image is located at `[public]:disk_image/dt-raspios-bullseye-lite-v3.0.8-arm64v8.zip`.

## Operations

The following operations are performed on the input image:
- Image is flashed to a virtual SD card and surgery is performed with name `amelia`;
- The repository `https://github.com/duckietown/workspace-brown2022-duckiedrone` is cloned at `/code/brown2022`;
- The Docker image `docker.io/duckietown/env-brown2022-aux:latest-arm64v8` is pulled onto the image;
- Secret tokens are cleared from the image;

## How to generate the image

### Environment setup

In order to execute arbitrary commands on the sd card image environment we need to enable multi-architecture support (in the typical use case our host will be an `amd64` machine, but the Duckiedrone uses a Raspberry Pi with `arm64v8` architecture.).

To enable this we need to install the following packages on our host:

```
qemu
binfmt-support
qemu-user-static
```

On Ubuntu this can be done by running:

```
sudo apt install qemu binfmt-support qemu-user-static
```

Then we need to run a qemu docker container:

```bash
docker run --rm --privileged multiarch/qemu-user-static:register --reset
```
### Base image retrieval
Download the base image with the following command:

```shell
dts data get --space public disk_image/dt-raspios-bullseye-lite-v3.0.8-arm64v8.zip ./dt-raspios-bullseye-lite-v3.0.8-arm64v8.zip
```

### Image generation
Run the following command to prepare a copy of the image (`VERSION_NUMBER` being an integer):

```shell
./generate.sh "<base_image>.img" "<VERSION_NUMBER>"
```

## How to push the image to DCSS

Run the following command to push the final image to the public space on the DCSS.
You might need to ask for permissions before you can push to this space.
Use the following command to push the final image:

```shell
dts data push ./dt-amelia-DD24-brown2022-sd-card-vXX.zip public:brown/disk_image/dt-amelia-DD24-brown2022-sd-card-vXX.zip
```