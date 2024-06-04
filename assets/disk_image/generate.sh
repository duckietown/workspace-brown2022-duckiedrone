#!/usr/bin/env bash

set -ex

INPUT_IMG=$1
OUT_VERSION=$2
SD_CARD_FILE=./dt-amelia-DD24-brown2022-sd-card-${OUT_VERSION}.img
SD_CARD_ZIP_FILE=./dt-amelia-DD24-brown2022-sd-card-${OUT_VERSION}.zip
ROOT_MOUNTPOINT=./partitions/root
CONFIG_MOUNTPOINT=./partitions/config
REGISTRY=docker.io
DISTRO=ente
ARCH=arm64v8

# perform surgery
dts init_sd_card \
    --type duckiedrone \
    --configuration DD24 \
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

# Install rake using chroot
sudo chroot --userspec=0:0 "${ROOT_MOUNTPOINT}" /bin/bash -c "apt update && apt install -y rake"

# copy custom autoboot compose file
sudo chown 1000:1000 ${ROOT_MOUNTPOINT}/data/autoboot
cp -f ./rootfs/data/autoboot/duckiedrone.yaml ${ROOT_MOUNTPOINT}/data/autoboot/duckiedrone.yaml

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
# strings -t d "${SD_CARD_FILE}" | grep "dt1-"

# clear
sudo rmdir ${ROOT_MOUNTPOINT}
sudo rmdir ${CONFIG_MOUNTPOINT}
sudo rmdir ./partitions

# compress
zip "${SD_CARD_ZIP_FILE}" "${SD_CARD_FILE}"

set +ex
