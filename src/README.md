# mechArm 270 M5 MoveIt 2 프로젝트

mechArm 270 M5 (카메라 플랜지 + 적응형 그리퍼) 를 위한  
ROS 2 Humble + MoveIt 2 프로젝트입니다.

---

## 패키지 구조

```
mecharm_moveit/
├── mecharm_description/          # URDF 및 시각화
│   ├── urdf/mecharm.urdf
│   └── launch/display.launch.py
│
├── mecharm_moveit_config/        # MoveIt 2 설정
│   ├── config/
│   │   ├── mecharm.srdf          # 플래닝 그룹, 충돌 설정
│   │   ├── kinematics.yaml       # KDL IK 솔버
│   │   ├── joint_limits.yaml     # 조인트 속도/가속도 제한
│   │   ├── ompl_planning.yaml    # OMPL 플래너 설정
│   │   ├── ros2_controllers.yaml # ros2_control 컨트롤러
│   │   ├── moveit_controllers.yaml
│   │   └── moveit.rviz
│   └── launch/
│       ├── demo.launch.py        # 시뮬레이션 (로봇 없이)
│       ├── real_robot.launch.py  # 실제 로봇 연결
│       ├── move_group.launch.py  # move_group 만
│       └── rviz.launch.py        # RViz 만
│
└── mecharm_hardware/             # pymycobot 하드웨어 드라이버
    └── mecharm_hardware/
        ├── mecharm_driver.py     # 실제 로봇 통신 노드
        └── mecharm_moveit_example.py  # MoveIt Python 예제
```

---

## 플래닝 그룹

| 그룹명   | 조인트                                      | 설명            |
|----------|---------------------------------------------|-----------------|
| `arm`    | joint1~joint6                               | 6축 플래닝      |
| `gripper`| gripper_controller                          | 그리퍼 제어     |

### 미리 정의된 포즈

| 포즈명          | 그룹    | 설명                  |
|-----------------|---------|-----------------------|
| `home`          | arm     | 모든 조인트 0°         |
| `init`          | arm     | 초기 준비 자세         |
| `gripper_open`  | gripper | 그리퍼 완전 열림       |
| `gripper_close` | gripper | 그리퍼 완전 닫힘       |

---

## 설치

### 1. 의존성 설치

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-ros-visualization \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui

pip install pymycobot
```

### 2. 워크스페이스 빌드

```bash
# 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 패키지 복사 (이 폴더 전체를 src/ 에 복사)
cp -r mecharm_moveit/* .

# mycobot_description 메시 파일 설치
# (기존 mycobot_description 패키지가 있다면 해당 경로 사용)
# 없다면: https://github.com/elephantrobotics/mycobot_ros 클론
git clone https://github.com/elephantrobotics/mycobot_ros.git

# 빌드
cd ~/ros2_ws
colcon build --packages-select \
  mecharm_description \
  mecharm_moveit_config \
  mecharm_hardware

source install/setup.bash
```

---

## 실행 방법

### 시뮬레이션 (로봇 없이 RViz 확인)

```bash
ros2 launch mecharm_moveit_config demo.launch.py
```

RViz 에서 **MotionPlanning** 패널 → Planning Group: `arm` 선택  
→ 인터랙티브 마커로 목표 설정 → **Plan & Execute**

### URDF 만 확인

```bash
ros2 launch mecharm_description display.launch.py
```

### 실제 로봇 연결

```bash
# 포트 확인
ls /dev/ttyUSB* /dev/ttyACM*

# 포트 권한 부여
sudo chmod 666 /dev/ttyUSB0

# 실행 (포트 지정)
ros2 launch mecharm_moveit_config real_robot.launch.py port:=/dev/ttyUSB0 baud:=115200
```

### Python 예제 실행 (실제 로봇 연결 후)

```bash
# 별도 터미널에서
source ~/ros2_ws/install/setup.bash
python3 src/mecharm_hardware/mecharm_hardware/mecharm_moveit_example.py
```

---

## 아키텍처

```
[RViz / Python 코드]
        │
        ▼ MoveGroup Action
[move_group node]  ←── SRDF, kinematics, OMPL
        │
        ▼ FollowJointTrajectory / GripperCommand Action
[mecharm_driver]  ←── /joint_states 퍼블리시
        │
        ▼ pymycobot API (시리얼)
[mechArm 270 M5 실제 로봇]
```

---

## 주요 토픽 / 액션

| 이름 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 현재 조인트 상태 |
| `/robot_description` | `std_msgs/String` | URDF |
| `/move_group` | `moveit_msgs/action/MoveGroup` | MoveIt 플래닝+실행 |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 팔 궤적 실행 |
| `/gripper_controller/gripper_action` | `control_msgs/action/GripperCommand` | 그리퍼 제어 |

---

## 트러블슈팅

**`No robot model loaded` 오류**  
→ `mecharm_description` 패키지가 빌드되어 있는지 확인  
→ `source install/setup.bash` 재실행

**시리얼 포트 연결 안 됨**  
→ `sudo chmod 666 /dev/ttyUSB0` 또는 사용자를 `dialout` 그룹에 추가  
→ `sudo usermod -aG dialout $USER` (재로그인 필요)

**IK 실패 (Cartesian pose goal)**  
→ `kinematics.yaml` 의 `kinematics_solver_timeout` 증가  
→ 작업 공간(workspace) 내 목표인지 확인 (mechArm 270 최대 도달 반경 ~270mm)

**그리퍼 움직임이 MoveIt 에 반영 안 됨**  
→ `mimic joint` 는 MoveIt 플래너에서 자동으로 처리되지 않음  
→ `gripper_controller` 하나만 제어하면 실제 로봇에서 mimic 동작은 하드웨어가 처리

---

## 라이선스

Apache-2.0
