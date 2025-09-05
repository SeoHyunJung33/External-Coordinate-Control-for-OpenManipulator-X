# External-Coordinate-Control-for-OpenManipulator-X

pick_moveit.cpp 코드는 ros2 + moveit을 이용한 로봇팔 자동 제어 프로그램으로, 사용자가 입력한 좌표를 목표로 로봇팔을 이동시키고, 물체를 집은 후 다시 원위치로 복귀하는 시뮬레이션을 자동으로 수행하는 코드이다.

## 작동과정

1. 사용자에게 좌표 입력 받기 (x, y, z)
2. 해당 좌표로 로봇팔(arm)을 이동
3. 그리퍼(gripper)를 닫아 물체 집기
4. 로봇팔을 홈 위치(init)로 이동
5. 그리퍼를 열어 물체 놓기

<코드로 보는 작동 과정>

1. ROS 2 노드 초기화 및 생성

```bash
rclcpp::init(argc, argv);
auto const node = std::make_shared<rclcpp::Node>(
  "pick_and_place_input",
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
```

ROS 2 시스템을 초기화하고, `pick_and_place_input`이라는 이름의 노드를 생성한다. 이 노드는 이후 MoveIt 인터페이스와의 통신에 사용된다.

1. MoveIt 인터페이스 생성

```bash
using moveit::planning_interface::MoveGroupInterface;
auto arm = MoveGroupInterface(node, "arm");
auto gripper = MoveGroupInterface(node, "gripper");
```

`MoveGroupInterface`를 사용하여 로봇 팔(`arm`)과 그리퍼(`gripper`)의 제어 인터페이스를 생성한다. 이 인터페이스를 통해 목표 위치 설정, 경로 계획, 실행 등의 작업을 수행할 수 있다.

1. 사용자로부터 목표 좌표 입력 받기

```bash
double x, y, z;
std::cout << "집을 목표 좌표를 입력하세요 (x y z): ";
std::cin >> x >> y >> z;
```

사용자로부터 물체를 집을 목표 위치의 좌표를 입력받는다. 입력된 좌표는 이후 로봇 팔의 목표 위치로 사용된다.

1. 목표 위치 설정 및 경로 계획

```bash
arm.setPositionTarget(x, y, z);
arm.setGoalPositionTolerance(0.01);
arm.setPlanningTime(10.0);
arm.setMaxVelocityScalingFactor(0.5);
arm.setMaxAccelerationScalingFactor(0.5);
```

입력받은 좌표를 로봇 팔의 목표 위치로 설정하고, 경로 계획을 위한 파라미터를 설정한다. 이후 계획된 경로를 실행한다.

1. 그리퍼 제어 및 원위치 복귀

```bash
gripper.setNamedTarget("close");
gripper.move();
// ...
arm.setNamedTarget("init");
arm.move();
// ...
gripper.setNamedTarget("open");
gripper.move();
```

그리퍼를 닫아 물체를 집고, 로봇 팔을 초기 위치로 이동시킨 후, 그리퍼를 열어 물체를 놓는다. 이러한 일련의 동작을 통해 픽 앤 플레이스 작업을 완료한다.

## 객체 인식을 통해 얻은 좌표를 변환하여 pick_moveit.cpp에 적용하는 방법

1. **객체 인식**: 카메라를 통해 객체를 인식하고, 이미지 좌표를 얻는다.
2. **깊이 정보 획득**: RGB-D 카메라를 사용하여 각 픽셀의 깊이 정보를 획득한다.
3. **좌표 변환**: 이미지 좌표와 깊이 정보를 사용하여 3D 포인트를 계산하고, 카메라의 내부 파라미터를 이용해 카메라 좌표계로 변환한다.
4. **로봇 좌표계로 변환**: TF(Transform) 프레임워크를 사용하여 카메라 좌표계를 로봇의 기준 좌표계로 변환한다.

이러한 과정을 통해 얻은 좌표를 `setPositionTarget(x, y, z)` 함수에 전달하여 로봇 팔이 정확한 위치로 이동할 수 있도록 한다.

## 코드 내 moveit 통신 흐름 분석

1. `move_group_interface`는 `move_group` 노드와 통신하는 여러 서비스/액션/토픽 핸들을 초기화한다.

```bash
// 1. 인터페이스 객체 생성
auto arm = MoveGroupInterface(node, "arm");
```

1. 내부적으로 역기구학(kinematics)을 계산하고 ROS 2 서비스 `/get_position_ik` 등을 사용하여 이 위치로 이동 가능한 joint 값 계산한다.이 단계에서 "목표" 상태가 설정되며, 경로는 아직 생성되지 않는다.

```bash
// 2. 목표 위치 설정
arm.setPositionTarget(x, y, z);
```

1. `move_group` 서비스 노드와 통신하여 실제 joint trajectory를 계산한다.

```bash
// 3. 경로 계획
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (arm.plan(plan) == MoveItErrorCode::SUCCESS);
```

1. `execute()`는 `/execute_trajectory` 액션 서버에 목표 trajectory를 전송한다. `move_group`가 이를 처리해 `/arm_controller/follow_joint_trajectory`에 실제 명령을 전달한다. `controller_manager`와 연결된 실제 제어기(controller)가 로봇을 움직인다.

---

```bash
// 4. 계획된 경로 실행
arm.execute(plan);
```

1. `gripper` 그룹도 `MoveGroupInterface`로 설정되었기 때문에, 그리퍼 동작은 preset 상태 (`"open"`, `"close"`)로 설정 후 실행된다. 내부적으로 `/gripper_controller/gripper_cmd` 액션 사용한다. 이름 기반 target은 SRDF 내 `group_states`에서 정의되어 있어야 한다.
