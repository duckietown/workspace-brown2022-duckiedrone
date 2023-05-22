#!/usr/bin/env bash

set -ex

INPUT_IMG=$1
OUT_VERSION=$2
SD_CARD_FILE=./dt-amelia-DD21-brown2022-sd-card-${OUT_VERSION}.img
SD_CARD_ZIP_FILE=./dt-amelia-DD21-brown2022-sd-card-${OUT_VERSION}.zip
ROOT_MOUNTPOINT=./partitions/root
CONFIG_MOUNTPOINT=./partitions/config
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
# strings -t d "${SD_CARD_FILE}" | grep "dt1-"

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
) | sudo fdisk "${LOOPDEV}"
sudo mkfs -t vfat "${LOOPDEV}p3"
sudo fatlabel "${LOOPDEV}p3" config

# mount partition
sudo mkdir -p ${ROOT_MOUNTPOINT}
sudo mount -t auto "${LOOPDEV}p2" ${ROOT_MOUNTPOINT}

# add default configuration
sudo mkdir -p ${CONFIG_MOUNTPOINT}
sudo mount -t auto "${LOOPDEV}p3" ${CONFIG_MOUNTPOINT}
sudo cp ./config/* ${CONFIG_MOUNTPOINT}/
sudo umount ${CONFIG_MOUNTPOINT}

# add /config to /etc/fstab
echo "/dev/mmcblk0p3  /config           vfat    defaults,flush    0       2" | sudo tee -a "${ROOT_MOUNTPOINT}/etc/fstab"

# clone workspace repository
sudo chown 1000:1000 ${ROOT_MOUNTPOINT}/code
git clone \
    https://github.com/duckietown/workspace-brown2022-duckiedrone \
    ${ROOT_MOUNTPOINT}/code/brown2022
git -C ${ROOT_MOUNTPOINT}/code/brown2022 status

# copy nodes' configuration files
sudo mkdir ${ROOT_MOUNTPOINT}/data/config/nodes
sudo chown 1000:1000 ${ROOT_MOUNTPOINT}/data/config/nodes
cp -R ./rootfs/data/config/nodes/* ${ROOT_MOUNTPOINT}/data/config/nodes/

# run dind
docker run \
    -d \
    --rm \
    --privileged \
    --name brown2022-disk-image-dind \
    -v "$(realpath ${ROOT_MOUNTPOINT}/var/lib/docker):/var/lib/docker" \
    docker:20.10.5-dind \
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

# log wifi-access-point by default
sudo mkdir ${ROOT_MOUNTPOINT}/data/logs/containers/wifi-access-point

# flush
sudo sync

# umount
sudo umount ${ROOT_MOUNTPOINT}
sudo losetup -d "${LOOPDEV}"

# find secrets
# strings -t d "${SD_CARD_FILE}" | grep "dt1-"

# clear
sudo rmdir ${ROOT_MOUNTPOINT}
sudo rmdir ${CONFIG_MOUNTPOINT}
sudo rmdir ./partitions

# compress
zip "${SD_CARD_ZIP_FILE}" "${SD_CARD_FILE}"

set +ex

