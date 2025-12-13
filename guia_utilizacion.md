# Guía rápida de uso – ROS 2 Humble + CoppeliaSim (Proyecto Mapas)

## 1. Visión general
El sistema separa **simulación** y **control**:
- **CoppeliaSim (local, con GUI)** ejecuta la escena y los sensores (sonares + Hokuyo).
- **ROS 2 Humble (en Docker)** ejecuta SLAM, control Bug2, gestión de metas e interfaz.
Se usa Docker para garantizar **entorno reproducible** y evitar problemas de dependencias; CoppeliaSim se mantiene en local por estabilidad gráfica y facilidad de depuración.

---

## 2. Paquetes ROS 2 utilizados (MUY IMPORTANTE)
Estos son los paquetes clave del sistema y **deben mencionarse explícitamente en la documentación**:

- **slam_toolbox**  
  👉 *Paquete principal usado para la creación del mapa de ocupación.*  
  Se utiliza el nodo `async_slam_toolbox_node`, que:
  - Consume `/scan` (LaserScan)
  - Publica `/map` (OccupancyGrid)
  - Publica TF `map → odom`  
  Es el núcleo del mapeo SLAM en esta práctica.

- **nav2_map_server**  
  👉 *Paquete usado para guardar el mapa en disco.*  
  Se usa la herramienta `map_saver_cli` para generar:
  - Archivo `.pgm` (imagen del mapa)
  - Archivo `.yaml` (metadatos del mapa)

- **tf2_ros**  
  Para publicar transformaciones estáticas (`static_transform_publisher`) y manejar el árbol TF.

- **geometry_msgs, nav_msgs, sensor_msgs**  
  Mensajes estándar para control, odometría, LIDAR y sonares.

- **entrega_mapas_package** (paquete propio)
  Contiene:
  - `coppelia_interface_node`
  - `bug2_controller_node`
  - `goal_manager_node`
  - Launch principal del sistema

---

## 3. Nodos principales y función
- **coppelia_interface_node**  
  Conecta con CoppeliaSim vía ZMQ. Publica `/robot/pose`, `/odom`, `/scan`, `/robot/sonar_*`, `/tf`. Consume `/cmd_vel`.
- **slam_toolbox (async)**  
  Construye el mapa de ocupación en tiempo real. Publica `/map`.
- **goal_manager_node**  
  Genera y publica metas en `/goal` (QoS TRANSIENT_LOCAL). Regenera metas al alcanzarlas.
- **bug2_controller_node**  
  Implementa el algoritmo Bug2. Lee pose, meta y sonares. Publica `/cmd_vel`.
- **static_transform_publisher**  
  Publica TF fijo `base_link → laser`.

---

## 4. Docker: build y acceso
Compilar imagen:
- `docker compose build`

Arrancar contenedor:
- `docker compose up -d`

Entrar al contenedor:
- `docker compose exec <servicio_ros> bash`

---

## 5. Compilación del workspace ROS 2
Dentro del contenedor:
- `source /opt/ros/humble/setup.bash`
- `cd /ros2_ws`
- `colcon build --symlink-install`
- `source /ros2_ws/install/setup.bash`

---

## 6. Ejecución del sistema
Lanzar todo el sistema (interfaz + SLAM + control):
- `ros2 launch entrega_mapas_package <archivo>.launch.py`

---

## 7. Conectividad con CoppeliaSim
- CoppeliaSim debe tener activo **ZMQ Remote API**.
- Docker se ejecuta con **network_mode: host** para permitir conexión por `localhost`.
- CoppeliaSim actúa como servidor local; ROS 2 se conecta desde el contenedor.

---

## 8. Inspección y diagnóstico
Estructura de paquetes:
- `ls /ros2_ws/src`
- `ros2 pkg executables entrega_mapas_package`

Nodos y grafo:
- `ros2 node list`
- `rqt_graph`

Topics y tipos:
- `ros2 topic list`
- `ros2 topic list -t`
- `ros2 topic echo /robot/sonar_4 --once`

---

## 9. Topics clave (contrato del sistema)
- `/cmd_vel` – comando de velocidad del robot.
- `/robot/pose` – pose del robot (desde CoppeliaSim).
- `/robot/sonar_1..16` – sensores de proximidad (Bug2).
- `/scan` – LIDAR (entrada de SLAM).
- `/odom`, `/tf`, `/tf_static` – cinemática y transformaciones.
- `/goal` – meta actual generada automáticamente.
- `/map` – mapa de ocupación generado por **slam_toolbox**.

---

## 10. Guardar el mapa (YAML + PGM) – MUY IMPORTANTE
Crear carpeta destino:
- `mkdir -p /ros2_ws/maps`

Guardar mapa usando **nav2_map_server**:
- `ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/mapa_practica`

Si no recibe `/map` por QoS:
- `ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/mapa_practica --ros-args -p map_subscribe_transient_local:=true`

Resultado generado:
- `mapa_practica.yaml`
- `mapa_practica.pgm`

---

## 11. Nota crítica sobre sonares (lección aprendida)
Para que los sonares funcionen correctamente en CoppeliaSim:
- Los muros/objetos **DEBEN** tener activada la propiedad **Detectable**.
- Si no, los sonares devuelven siempre la distancia máxima y el robot no detecta obstáculos.

---

## 12. Checklist rápido
- ¿Hay pose? → `ros2 topic echo /robot/pose --once`
- ¿Sonares varían al acercarse a muros? → `ros2 topic echo /robot/sonar_4 --once`
- ¿El mapa se publica? → `ros2 topic echo /map --once`
- ¿Mapa guardado correctamente? → comprobar `/ros2_ws/maps`

---

## 13. Entregables finales habituales
- Carpeta `maps/` con archivos `.yaml` y `.pgm`.
- Código del paquete `entrega_mapas_package`.
- Documentación indicando explícitamente:
  - Uso de **slam_toolbox** para SLAM.
  - Uso de **nav2_map_server** para guardado del mapa.
