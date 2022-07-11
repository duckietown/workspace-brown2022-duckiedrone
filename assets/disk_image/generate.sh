#!/usr/bin/env bash

set -ex

INPUT_IMG=$1
SD_CARD_FILE=./dt-amelia-DD21-brown2022-sd-card-v1.img
SD_CARD_ZIP_FILE=./dt-amelia-DD21-brown2022-sd-card-v1.zip
ROOT_PARTITION=rootfs
ROOT_MOUNTPOINT=./partitions/root
REGISTRY=docker.io
DISTRO=ente
ARCH=arm64v8

# perform surgery
dts init_sd_card \
    --type duckiedrone \
    --configuration DD21 \
    --hostname amelia \
    --workdir "$(dirname $INPUT_IMG)" \
    --experimental \
    --device "${SD_CARD_FILE}"

# find secrets
strings -t d "${SD_CARD_FILE}" | grep "dt1-"

# mount disk
LOOPDEV=$(sudo losetup --show -fPL "${SD_CARD_FILE}")
sudo udevadm trigger

# make /config partition
(
echo n    # Add a new partition
echo p    # Primary partition
echo 3    # Partition number
echo      # First sector (Accept default: 2048)
echo      # Last sector (Accept default: 8191)
echo t    # Change partition type
echo 3    # Partition number
echo 0c   # Partition type FAT32 (LBA)
echo w    # Write changes
) | sudo fdisk ${LOOPDEV}
sudo mkfs -t vfat ${LOOPDEV}p3

# mount partition
DISK=/dev/disk/by-label/${ROOT_PARTITION}
sudo mkdir -p ${ROOT_MOUNTPOINT}
sudo mount -t auto ${DISK} ${ROOT_MOUNTPOINT}

# add /config to /etc/fstab
sudo mkdir "${ROOT_MOUNTPOINT}/config"
echo "PARTUUID=69aeb770-03  /config           vfat    defaults,flush    0       2" >> "${ROOT_MOUNTPOINT}/etc/fstab"

# clone workspace repository
sudo git clone \
    https://github.com/duckietown/workspace-brown2022-duckiedrone \
    ${ROOT_MOUNTPOINT}/code/brown2022
sudo chown -R 1000:1000 ${ROOT_MOUNTPOINT}/code/brown2022

# run dind
docker run \
    -d \
    --rm \
    --privileged \
    --name brown2022-disk-image-dind \
    -v "$(realpath ${ROOT_MOUNTPOINT}/var/lib/docker):/var/lib/docker" \
    docker:dind \
        dockerd --host=tcp://0.0.0.0:2375 --bridge=none

# wait for dind to start and then get its IP
sleep 20
DIND_IP=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' brown2022-disk-image-dind)

# pull image
docker -H "tcp://${DIND_IP}:2375" pull docker.io/duckietown/env-brown2022-aux:latest-arm64v8

# update all images
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-device-dashboard:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-files-api:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-code-api:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-device-proxy:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-device-health:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-device-online:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-vscode:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-wifi-access-point:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-ros-commons:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-duckiebot-interface:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-drone-interface:${DISTRO}-${ARCH}
docker -H "tcp://${DIND_IP}:2375" pull ${REGISTRY}/duckietown/dt-rosbridge-websocket:${DISTRO}-${ARCH}

# remove old images
docker -H "tcp://${DIND_IP}:2375" image prune --force

# stop dind
docker stop brown2022-disk-image-dind

# remove secrets
ls -alh ${ROOT_MOUNTPOINT}/secrets/tokens/dt1
sudo shred ${ROOT_MOUNTPOINT}/secrets/tokens/dt1
sudo rm -f ${ROOT_MOUNTPOINT}/secrets/tokens/dt1

# flush
sudo sync

# umount
sudo umount ${ROOT_MOUNTPOINT}
sudo losetup -d "${LOOPDEV}"

# find secrets
strings -t d "${SD_CARD_FILE}" | grep "dt1-"

# clear
sudo rmdir ${ROOT_MOUNTPOINT}
sudo rmdir ./partitions

# compress
zip "${SD_CARD_ZIP_FILE}" "${SD_CARD_FILE}"

set +ex
