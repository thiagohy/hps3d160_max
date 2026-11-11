#!/usr/bin/env bash
# Run the HPS-3D160 system inside the ROS 1 Noetic container.
#
#   ./scripts/ros_docker.sh build      build the catkin workspace
#   ./scripts/ros_docker.sh shell      interactive shell with ROS sourced
#   ./scripts/ros_docker.sh node       roscore + camera node (background)
#   ./scripts/ros_docker.sh rviz       RViz with the pre-configured point cloud
#   ./scripts/ros_docker.sh validate   publish, record a bag and inspect the cloud
#   ./scripts/ros_docker.sh stop       stop the background node
#
# The device is passed through as /dev/ttyACM0 (adjust with HPS3D_PORT).
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WORKSPACE="${ROS_WS:-$HOME/ros_ws}"
IMAGE="${ROS_IMAGE:-hps3d160-ros}"
PORT="${HPS3D_PORT:-/dev/ttyACM0}"
CONTAINER="${ROS_CONTAINER:-hps3d160_ros}"

COMMON_ARGS=(
  --rm
  --name "$CONTAINER"
  --user "$(id -u):$(id -g)"
  -e HOME=/tmp
  -e DISPLAY="${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -v /home/thiagohy:/home/thiagohy
  -v /tmp/.X11-unix:/tmp/.X11-unix
  --network host
)

build() {
  docker build -t "$IMAGE" -f "$PROJECT_DIR/docker/ros.Dockerfile" "$PROJECT_DIR"
  docker run "${COMMON_ARGS[@]}" -w "$WORKSPACE" "$IMAGE" \
    bash -lc "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release"
}

shell() {
  docker run -it "${COMMON_ARGS[@]}" -w "$WORKSPACE" "$IMAGE" bash
}

node() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  # --privileged is required so the container may open the USB/ACM serial node
  # (the Docker device cgroup blocks it otherwise).
  docker run -d --name "$CONTAINER" --privileged \
    --user 0:0 -e HOME=/tmp -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    -v /home/thiagohy:/home/thiagohy \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PORT:$PORT" \
    --network host \
    -w "$WORKSPACE" "$IMAGE" \
    bash -lc "source /opt/ros/noetic/setup.bash
              source $WORKSPACE/devel/setup.bash
              roscore >/tmp/roscore.log 2>&1 &
              sleep 4
              rosrun hps3d160_max hps3d160_max_node --port $PORT --max-frames 0"
  echo "container $CONTAINER started (logs: docker logs -f $CONTAINER)"
}

rviz() {
  docker exec -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    "$CONTAINER" bash -lc \
    "source /opt/ros/noetic/setup.bash && source $WORKSPACE/devel/setup.bash && \
     rviz -d $PROJECT_DIR/config/hps3d160.rviz"
}

validate() {
  docker exec "$CONTAINER" bash -lc \
    "source /opt/ros/noetic/setup.bash && source $WORKSPACE/devel/setup.bash && \
     echo '--- topic info ---' && rosrun rqt_topicinfo rqt_topicinfo /hps3d160/point_cloud 2>/dev/null || rostopic info /hps3d160/point_cloud; \
     echo '--- rate ---' && timeout 6 rostopic hz /hps3d160/point_cloud; \
     echo '--- message ---' && rostopic echo -n 1 --noarr /hps3d160/point_cloud | head -40"
}

stop() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  echo "container stopped"
}

case "${1:-build}" in
  build) build ;;
  shell) shell ;;
  node) node ;;
  rviz) rviz ;;
  validate) validate ;;
  stop) stop ;;
  *) echo "usage: $0 {build|shell|node|rviz|validate|stop}" >&2; exit 1 ;;
esac
