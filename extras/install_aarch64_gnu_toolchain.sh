#!/bin/bash

set -euo pipefail

# VERSION="11.3.rel1"
# GNU_TOOLCHAIN_NAME="arm-gnu-toolchain-${VERSION}-x86_64-aarch64-none-linux-gnu"
# VERSION="10.3-2021.07"
VERSION="10.2-2020.11"
GNU_TOOLCHAIN_NAME="gcc-arm-${VERSION}-x86_64-aarch64-none-linux-gnu"
INSTALL_DIR="/usr/local/aarch64-none-linux-gnu"

rm -rf "${INSTALL_DIR}"
mkdir "${INSTALL_DIR}"
chmod 755 "${INSTALL_DIR}"
cd "${INSTALL_DIR}"
# wget "https://developer.arm.com/-/media/Files/downloads/gnu/${VERSION}/binrel/${GNU_TOOLCHAIN_NAME}.tar.xz"
wget "https://developer.arm.com/-/media/Files/downloads/gnu-a/${VERSION}/binrel/${GNU_TOOLCHAIN_NAME}.tar.xz"
tar xf "${GNU_TOOLCHAIN_NAME}.tar.xz"
mv "${GNU_TOOLCHAIN_NAME}"/* .
rm -r "${GNU_TOOLCHAIN_NAME}.tar.xz" "${GNU_TOOLCHAIN_NAME}"