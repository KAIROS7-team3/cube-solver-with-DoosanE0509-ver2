# cube_solver_ver2 (ROS2)

Doosan E0509 + RH-P12-RN(A) + RealSense D455 기반 루빅스 큐브 솔버 (ROS2 Humble).

## 저장소 받기

```bash
git clone <이 저장소 URL> cube_solver_ver2_ws
cd cube_solver_ver2_ws
```

Doosan 드라이버는 별도 클론 (`humble` 브랜치):

```bash
git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git src/doosan-robot2
```

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
