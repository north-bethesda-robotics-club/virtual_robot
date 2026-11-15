#!/usr/bin/env bash
# Build the robot code in ../TeamCode/robot and install it on a REV Control Hub.
#
# Usage:
#   ./deploy.sh          build + install (connects over Wi-Fi if no device is attached)
#   ./deploy.sh check    only compile against the real FTC SDK (no robot needed)
#
# Connecting:
#   - USB: plug the Control Hub's USB-C port into the computer.
#   - Wi-Fi: join the Control Hub's Wi-Fi network (e.g. "FTC-XXXX"); this script
#     then runs `adb connect 192.168.43.1:5555` for you.
set -euo pipefail
cd "$(dirname "$0")"

HUB_ADDR="${HUB_ADDR:-192.168.43.1:5555}"

# Gradle needs to know where the Android SDK is.
if [[ ! -f local.properties ]]; then
  sdk="${ANDROID_HOME:-${ANDROID_SDK_ROOT:-}}"
  if [[ -z "$sdk" ]]; then
    for candidate in "$HOME/Library/Android/sdk" "$HOME/Android/Sdk" "${LOCALAPPDATA:-}/Android/Sdk"; do
      [[ -d "$candidate" ]] && sdk="$candidate" && break
    done
  fi
  if [[ -z "$sdk" ]]; then
    echo "Can't find the Android SDK. Install Android Studio, or set ANDROID_HOME." >&2
    exit 1
  fi
  echo "sdk.dir=$sdk" > local.properties
fi

if [[ "${1:-}" == "check" ]]; then
  exec ./gradlew :TeamCode:compileDebugJavaWithJavac
fi

sdk_dir="$(sed -n 's/^sdk\.dir=//p' local.properties)"
adb="$(command -v adb || echo "$sdk_dir/platform-tools/adb")"

if ! "$adb" devices | grep -qw "device$"; then
  echo "No device attached; trying Control Hub over Wi-Fi at $HUB_ADDR ..."
  "$adb" connect "$HUB_ADDR" || true
  if ! "$adb" devices | grep -qw "device$"; then
    echo "Couldn't reach the Control Hub. Plug in USB-C or join its Wi-Fi network." >&2
    exit 1
  fi
fi

./gradlew :TeamCode:installDebug
echo
echo "Installed. On the Driver Station, pick your OpMode from the list."
