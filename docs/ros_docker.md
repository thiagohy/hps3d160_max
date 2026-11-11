# ROS 1 Noetic em Docker (WSL)

O host (Ubuntu 24.04 / WSL2) não tem binários de ROS 1 Noetic, então o build
catkin e a validação no RViz rodam em container.

```
┌─ WSL host ───────────────────────────────┐
│  /dev/ttyACM0  ← usbipd ← Windows (COM3)  │
│  ~/ros_ws/src/hps3d160_max -> projeto     │
│  X11 (:0 via WSLg)                         │
└───────────────┬───────────────────────────┘
                │  docker run --privileged --network host
┌───────────────▼───────────────────────────┐
│  ros:noetic-perception (+ rviz, xvfb)     │
│  roscore + hps3d160_max_node + rviz        │
└───────────────────────────────────────────┘
```

## Uso

```bash
./scripts/ros_docker.sh build      # constrói a imagem e compila o catkin
./scripts/ros_docker.sh node       # roscore + nó da câmera (background)
./scripts/ros_docker.sh rviz       # RViz com config/hps3d160.rviz
./scripts/ros_docker.sh validate   # rostopic hz/echo do cloud
./scripts/ros_docker.sh stop       # derruba o container
```

O container precisa de `--privileged` porque o *device cgroup* do Docker bloqueia
a abertura do `/dev/ttyACM0` (USB/IP do WSL) sem privilégios.

## Validação executada

| Item | Resultado |
|---|---|
| Build catkin (Noetic + PCL 1.12) | OK |
| Tópico `/hps3d160/point_cloud` | publicado, ~20 Hz |
| Geometria da mensagem | 160 × 60 (organized), `point_step=16`, `row_step=2560` |
| Campos | `x`, `y`, `z`, `distance` (float32) |
| Pixels válidos | 9.524 / 9.600 (99,2 %) |
| Erro radial \|r − distance\| | **0,000 mm** (mediana e p95) |
| Ângulo por coluna | col 0 = +37,57°, col 80 = 0,00°, col 159 = −37,22° |
| RViz | renderiza o cloud a 31 fps (captura em `docs/images/rviz_pointcloud.png`) |
| Bag | `rosbag record` → 232 mensagens / 11,4 s / 34 MB |

## Captura de tela do RViz (WSLg)

O X do WSLg não permite `xwd -root` (BadMatch em X_GetImage), então se captura
a **janela** do RViz:

```bash
docker exec hps3d160_ros bash -lc \
  "export DISPLAY=:0; xwininfo -root -tree | grep RViz"     # pega o window id
docker exec hps3d160_ros bash -lc \
  "export DISPLAY=:0; xwd -id 0x60000c -silent > /tmp/s.xwd && convert /tmp/s.xwd /tmp/s.png"
docker cp hps3d160_ros:/tmp/s.png ./shot.png
```

## Testes do publisher ROS

- `scripts/test_ros_publisher.sh` — valida o publisher contra um **mock** da API
  ROS (roda em qualquer host, sem ROS instalado)
- `tests/test_ros_publisher_real.cpp` — valida o publisher contra a **API real**
  do `sensor_msgs` (roda dentro do container)
