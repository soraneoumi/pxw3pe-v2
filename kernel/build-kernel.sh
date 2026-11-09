#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Builds a Rust-enabled kernel from kernel.org and installs it as Debian packages.
#
# Usage: build-kernel <version> [config]    (for example 6.18.60)
#
# Run as root from a login shell, so that rustup's rustc is in PATH. The source goes to
# /usr/src/linux-<version> and is checked against its release signature first. The configuration
# is the given file, or the running kernel's without one. rust-headers.patch, which lies next to
# this script, puts the Rust build artifacts into the headers package; DKMS can then rebuild its
# Rust modules when the packages are installed. The machine is not rebooted.

set -euo pipefail

readonly SRC=/usr/src
readonly CDN=https://cdn.kernel.org/pub/linux/kernel
# Keys that sign kernel.org releases (https://www.kernel.org/signature.html).
readonly SIGNERS=(
	647F28654894E3BD457199BE38DBBDC86092693E # Greg Kroah-Hartman
	ABAF11C65A2970B130ABE3C479BE3E4300411886 # Linus Torvalds
)
# Options the installed system needs. olddefconfig drops CONFIG_RUST without a word when the
# installed rustc or bindgen does not suit the new kernel, so they are checked after it.
readonly REQUIRED=(RUST DVB_CORE)

die() {
	echo "build-kernel: $*" >&2
	exit 1
}

[[ ($# -eq 1 || $# -eq 2) && $1 =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]] ||
	die "usage: build-kernel <version> [config]"
readonly VERSION=$1
readonly TREE=$SRC/linux-$VERSION
readonly PATCH=$(dirname "$(readlink -f "$0")")/rust-headers.patch
# Absolute, since the configuration is copied after changing into the tree.
readonly BASE_CONFIG=$(realpath -m "${2:-/boot/config-$(uname -r)}")

[[ $EUID -eq 0 ]] || die "must run as root"
command -v rustc >/dev/null || die "rustc is not in PATH; run from a root login shell"
[[ -f $PATCH ]] || die "$PATCH is missing"
[[ -f $BASE_CONFIG ]] || die "$BASE_CONFIG is missing"
grep -qx CONFIG_RUST=y "$BASE_CONFIG" || die "$BASE_CONFIG has no Rust support"
[[ ! -e $TREE ]] || die "$TREE already exists"

# Downloaded and prepared next to the final tree, so that a failure before the build leaves
# nothing behind and the finished tree moves into place with a rename.
work=$(mktemp -d "$SRC/.build-kernel.XXXXXX")
trap 'rm -rf "$work"' EXIT

url=$CDN/v${VERSION%%.*}.x/linux-$VERSION.tar
echo "Downloading linux-$VERSION"
curl -fsS --proto =https -o "$work/linux.tar.xz" "$url.xz"
curl -fsS --proto =https -o "$work/linux.tar.sign" "$url.sign"

# The signature covers the uncompressed tarball. The keys come from kernel.org's key directory
# into a keyring of this run only, and are trusted by their fingerprints alone.
export GNUPGHOME=$work/gnupg
mkdir -m 0700 "$GNUPGHOME"
gpg --batch --quiet --auto-key-locate clear,wkd \
	--locate-external-keys torvalds@kernel.org gregkh@kernel.org >/dev/null
xz -dc "$work/linux.tar.xz" |
	gpg --batch --status-fd 3 --verify "$work/linux.tar.sign" - 3>"$work/status" 2>/dev/null ||
	die "bad signature on linux-$VERSION.tar"
signer=$(awk '$2 == "VALIDSIG" { print $NF }' "$work/status")
[[ " ${SIGNERS[*]} " == *" $signer "* ]] || die "linux-$VERSION.tar is not signed by a release key"
echo "Signature good (key $signer)"

tar -xf "$work/linux.tar.xz" -C "$work"
patch -d "$work/linux-$VERSION" -p1 --forward --batch --quiet <"$PATCH"
mv "$work/linux-$VERSION" "$TREE"
cd "$TREE"

make LLVM=1 rustavailable
cp "$BASE_CONFIG" .config
make LLVM=1 olddefconfig >/dev/null
for option in "${REQUIRED[@]}"; do
	grep -Eq "^CONFIG_$option=[ym]$" .config || die "CONFIG_$option is off in the new configuration"
done
release=$(make -s LLVM=1 kernelrelease)

log=$SRC/build-$release.log
echo "Building $release (log: $log)"
make -j"$(nproc)" LLVM=1 bindeb-pkg >"$log" 2>&1 || die "build failed; see $log"

# debian/files lists the packages the build produced; the C library headers are left out,
# since they would replace the distribution's.
mapfile -t packages < <(awk '$1 ~ /^linux-(image|headers)-/ && $1 !~ /-dbg_/ { print $1 }' debian/files)
[[ ${#packages[@]} -eq 2 ]] || die "unexpected packages in $TREE/debian/files"
dpkg -i "${packages[@]/#/$SRC/}"

echo "Installed $release; it boots by default after the next reboot."
