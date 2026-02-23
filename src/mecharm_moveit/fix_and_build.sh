#!/bin/bash
# fix_and_build.sh
# CMake 캐시 충돌 해소 + 빌드 스크립트
# 실행: bash fix_and_build.sh (~/cobot_ws 에서)

set -e
WS_DIR="$HOME/cobot_ws"
cd "$WS_DIR"

echo "=== [1] 기존 mecharm 관련 빌드 캐시 삭제 ==="
rm -rf build/mecharm_description
rm -rf build/mecharm_moveit_config
rm -rf build/mecharm_hardware
rm -rf install/mecharm_description
rm -rf install/mecharm_moveit_config
rm -rf install/mecharm_hardware
echo "Done."

echo ""
echo "=== [2] 중복 mecharm_description 확인 ==="
# src 아래에 mecharm_description 이 2곳 이상 있으면 충돌
COUNT=$(find src -maxdepth 3 -name "package.xml" -exec grep -l "mecharm_description" {} \; | wc -l)
echo "Found mecharm_description package.xml count: $COUNT"
find src -maxdepth 3 -name "package.xml" -exec grep -l "mecharm_description" {} \;

echo ""
echo "=== [3] 빌드 ==="
colcon build --symlink-install \
  --packages-select mecharm_description mecharm_moveit_config mecharm_hardware \
  --event-handlers console_direct+

echo ""
echo "=== [4] source ==="
echo "Run: source install/setup.bash"
