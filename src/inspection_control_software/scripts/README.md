# Panel de Control del Robot — Scripts Python

Aplicación de escritorio en **PyQt5** para controlar y supervisar un robot
móvil de inspección sobre **ROS1**: carga/creación de mapas, teleoperación,
navegación autónoma (`move_base`/`amcl`), programación de patrullajes,
captura de fotos de referencia en puntos de interés, detección de alertas
(obstrucciones, bloqueos de camino, robot atascado) y un historial de
alertas con mapas de calor y estadísticas.

Este documento resume la arquitectura para orientarse en el código; cada
módulo tiene además un docstring de cabecera y docstrings en sus clases y
métodos principales.

## Punto de entrada

[`main.py`](main.py) inicializa el nodo ROS y lanza la ventana principal
(`App`), que monta tres pestañas:

- **Home** → [`home_view.py`](home_view.py)
- **Configuración** → [`configuration.py`](configuration.py)
- **Sistema de alertas** → [`logsystem_view.py`](logsystem_view.py)

Las tres comparten una única instancia de
`NodesManager` ([`config_model.py`](config_model.py)), responsable de
lanzar/detener los nodos de ROS (`roslaunch`) que usa cada pantalla.

## Módulos principales

### Core / infraestructura

| Módulo | Responsabilidad |
|---|---|
| [`config_model.py`](config_model.py) | `NodesManager` (lanzar/parar nodos ROS), `ConfigModel`/`UserConfigFileManager` (parámetros ROS y config de usuario persistidos en YAML/JSON). |
| [`database_manager.py`](database_manager.py) | `InternalStorageManager`: todo el CRUD contra PostgreSQL (SQLAlchemy) para patrullajes, checkpoints, mapas, alertas y calibración. `DataBase` lo envuelve en un `QThread` para no bloquear la UI. |
| [`internal_storage/`](internal_storage) | Modelos SQLAlchemy (`tables.py`) y gestor interno de almacenamiento. |

### Pestaña Home

| Módulo | Responsabilidad |
|---|---|
| [`home_view.py`](home_view.py) | `HomePanel` ensambla `VisualizationPanel` (mapa/cámara, carga/creación de mapa, marcadores interactivos), `SelectModePanel` (Auto/Manual/Localizar) y `PatrolsPanel` (CRUD + ejecución de patrullajes). También `BatteryIndicator`. |
| [`better_image_display.py`](better_image_display.py) | Editor de checkpoints sobre el mapa: colocar, mover, orientar (yaw) y eliminar puntos de interés con el mouse. |
| [`robot_camera_view.py`](robot_camera_view.py) | Vista de cámara en vivo con el recuadro de ROI, y captura de fotos de referencia en checkpoints. |
| [`image_carousel.py`](image_carousel.py) | Carrusel paginado de fotos de referencia/drenaje. |
| [`navbar.py`](navbar.py) | Barra superior con logos. |

### Patrullajes

| Módulo | Responsabilidad |
|---|---|
| [`patrols_scheduler.py`](patrols_scheduler.py) | `ScheduleChecker` (hilo que vigila el reloj y despacha el patrullaje que corresponde) + `PatrolsEscheduler` (API usada por la UI: CRUD de patrullajes, persistencia en BD). |
| [`points_scheduler.py`](points_scheduler.py) | `PointsScheduler`: envía cada checkpoint a `move_base`, corre la "subrutina" de inspección (comparación de similitud de imagen contra la foto de referencia) y gestiona el retorno a casa por batería baja. |
| [`patrol_menu.py`](patrol_menu.py) | Popup para elegir días/hora de un patrullaje. |
| [`place_form.py`](place_form.py) | Formulario de selección de carpeta al guardar un mapa. |

### Robot / ROS

| Módulo | Responsabilidad |
|---|---|
| [`robot_navigation_checker.py`](robot_navigation_checker.py) | `RobotNavigationChecker` (detecta cambios bruscos en el plan global → posible bloqueo) y `StuckDetector` (detecta robot atascado comparando `cmd_vel` vs odometría). |
| [`pose_controller.py`](pose_controller.py) | Controlador PID diferencial para acercarse a un marcador ArUco. |
| [`robot_control_vel.py`](robot_control_vel.py) | Panel de modo de conducción (Suave/Media/Agresiva) y tolerancias de navegación. |
| [`robot_form.py`](robot_form.py) | Formulario de perfiles de robot (URDF, tópicos). |
| [`robot_actions_logger.py`](robot_actions_logger.py) | Logger de acciones de la app: terminal en la UI + CSV (`logger` global). |
| [`manual_control_node.py`](manual_control_node.py) | Teleoperación tipo joystick → `cmd_vel`. |
| [`robot_vision.py`](robot_vision.py) | Experimentos de emparejamiento de features ORB (no integrado al pipeline). |

### Alertas y mapas de calor

| Módulo | Responsabilidad |
|---|---|
| [`alert_generator.py`](alert_generator.py) | `AlertGenerator`: construye una alerta (mensaje + pose + foto) y la persiste en BD. |
| [`heatmap_generator.py`](heatmap_generator.py) / [`generate_heatmap.py`](generate_heatmap.py) | Generan el mapa de calor de alertas por interpolación RBF sobre el mapa. |
| [`heatmap.py`](heatmap.py) | Script experimental independiente (no usado por el resto de la app). |
| [`logsystem_view.py`](logsystem_view.py) | Pestaña "Sistema de alertas": selector de mapa, histograma, lista paginada/filtrable de alertas, panel de estadísticas y mapa de calor. |

### UI auxiliar

[`custom_tooltip.py`](custom_tooltip.py), [`input_textdialog.py`](input_textdialog.py),
[`notification.py`](notification.py), estilos en [`styles/`](styles) y widgets
reutilizables en [`utils/`](utils) (`alert.py`, `patrol.py` con los `Enum`
compartidos, `sorting.py`, etc.).

## Notas

- La UI está en español; el código y comentarios mezclan español/inglés.
- No hay suite de tests automatizados para este paquete (los archivos
  `test_*.py` en este directorio son scripts de prueba manual/exploración,
  no tests unitarios).
- Varias funciones/métodos documentados están marcados como "no-op",
  "stub" o "código inalcanzable": se dejaron así intencionalmente durante
  esta tarea de documentación (no se modificó comportamiento).
