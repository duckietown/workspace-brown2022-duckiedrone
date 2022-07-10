# Generate a disk image for brown2022

The following tutorial explains how the disk image for `brown2022` is created.

## Base (input) image

The base image for this course is `dt-raspios-bullseye-lite-v3.0.0-arm64v8.img`
and can be downloaded from the official duckietown repository on the DCSS.
The image is located at `[public]:disk_image/dt-raspios-bullseye-lite-v3.0.0-arm64v8.zip`.

## Operations

The following operations are performed on the input image:
- Image is flashed to a virtual SD card and surgery is performed with name `amelia`;
- The repository `https://github.com/duckietown/workspace-brown2022-duckiedrone` is cloned at `/code/brown2022`;
- The Docker image `docker.io/duckietown/env-brown2022-aux:latest-arm64v8` is pulled onto the image;
- Secret tokens are cleared from the image;

## How to generate the image

Run the following command to prepare a copy of the image:

```shell
./generate.sh "<base_image>.img"
```

## How to push the image to DCSS

Run the following command to push the final image to the public space on the DCSS.
You might need to ask for permissions before you can push to this space.
Use the following command to push the final image:

```shell
dts data push ./dt-amelia-DD21-brown2022-sd-card-v1.zip public:brown/disk_image/dt-amelia-DD21-brown2022-sd-card-v1.zip
```