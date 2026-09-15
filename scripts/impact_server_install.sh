#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo "Ubuntu 22.04 required" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "Install ROS 2 Humble first (see Chinese runbook)." >&2; exit 2; }
: "${ARDUPILOT_ROOT:?Set a dedicated ArduPilot clone path}"
: "${ARDUPILOT_GAZEBO_ROOT:?Set a dedicated plugin clone path}"
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg git build-essential cmake pkg-config \
 python3-colcon-common-extensions python3-rosdep python3-pytest python3-numpy python3-scipy \
 python3-matplotlib mesa-utils ffmpeg rapidjson-dev libopencv-dev \
 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
 ros-humble-mavros ros-humble-mavros-extras ros-humble-geographic-msgs \
 ros-humble-tf2-ros ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-rosbag2
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /tmp/impact-gazebo.gpg
sudo install -m 0644 /tmp/impact-gazebo.gpg /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n' "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/impact-gazebo.list
sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-sim8-dev
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update --rosdistro humble
rosdep install --from-paths "$root/src" --ignore-src --rosdistro humble -r -y

clone_locked() {
  local url="$1" ref="$2" target="$3"
  if [[ -e "$target" ]]; then
    expected="$(git -C "$target" rev-parse "${ref}^{commit}")"
    [[ "$(git -C "$target" rev-parse HEAD)" == "$expected" && -z "$(git -C "$target" status --porcelain)" ]] || {
      echo "Existing $target is not a clean requested revision; choose a fresh dedicated path." >&2; return 2; }
  else
    git clone "$url" "$target"
    git -C "$target" checkout --detach "$ref"
  fi
}
plugin_ref="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["ardupilot_gazebo"]["ref"])' "$root/config/impact_dependencies.json")"
clone_locked https://github.com/ArduPilot/ardupilot.git Copter-4.5.7 "$ARDUPILOT_ROOT"
git -C "$ARDUPILOT_ROOT" submodule update --init --recursive
(cd "$ARDUPILOT_ROOT"; Tools/environment_install/install-prereqs-ubuntu.sh -y)
export PATH="$HOME/.local/bin:$PATH"
(cd "$ARDUPILOT_ROOT"; ./waf configure --board sitl; ./waf copter)
clone_locked https://github.com/ArduPilot/ardupilot_gazebo.git "$plugin_ref" "$ARDUPILOT_GAZEBO_ROOT"
export GZ_VERSION=harmonic
cmake -S "$ARDUPILOT_GAZEBO_ROOT" -B "$ARDUPILOT_GAZEBO_ROOT/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$ARDUPILOT_GAZEBO_ROOT/build" --parallel "${IMPACT_BUILD_JOBS:-2}"
echo "Dependencies built. Run impact.sh build, test, doctor --runtime, validate-server next."
