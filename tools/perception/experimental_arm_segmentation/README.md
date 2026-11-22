# Separación experimental de nube del brazo

Esta herramienta preserva el prototipo C++/OpenGL de la sesión del 22 de
noviembre de 2026. Se ejecuta por separado y no manda comandos ni publica al
supervisor. Sus tres salidas son candidatos de visualización, no espacio libre
ni información validada para decisiones de seguridad.

El programa procesa directamente `/camera/d435i/depth/color/points`, conserva
el rango 0,10–1,50 m, aplica el ajuste visual de la sesión y usa TF de los
eslabones a la marca de captura. Clasifica como brazo puntos a menos de 20 mm
de la malla muestreada; como inciertos, los restantes a menos de 60 mm de la
malla y de 140 mm de un centro articular, o dentro de las cápsulas; el resto
forma la tercera nube. Las mallas se preservan en `mesh_points.bin`, una
muestra fija del robot de la sesión, no una geometría universal. Los radios se
leen de la copia congelada `jaco_capsules_experimental.yaml`, tomada de
`src/thesis_core/config/jaco_capsules.yaml` durante la sesión.

La matriz cámara–base está incorporada en `segmentation_node.cpp` y coincide
con `thesis_perception/config/d435i_overlay_experimental.yaml` al incorporarse
este prototipo. Es un ajuste **experimental, no una calibración**; no se
actualiza automáticamente si se edita el YAML. Revisar ambos y revalidar tras
cambiar cámara, robot, base, montaje, modelo o parámetros. El filtro no publica
TF ni detecta zonas ocultas u objetos de forma fiable.

## Compilación

Requiere el workspace de ROS 2 Humble compilado y sourced, CMake, yaml-cpp,
OpenGL 4.3 con GLX, GLEW y X11. El helper solicita NVIDIA PRIME render offload
y necesita una sesión gráfica y un driver compatibles. No instala dependencias.
Desde la raíz del proyecto:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cmake -S tools/perception/experimental_arm_segmentation \
  -B build/experimental_arm_segmentation -DCMAKE_BUILD_TYPE=Release
cmake --build build/experimental_arm_segmentation -j2
```

## Inicio de visualización

1. Tener una cámara y un `robot_state_publisher` con `/joint_states` y TF
   válidos. El procedimiento de cámara y superposición está en
   [`thesis_perception/README.md`](../../../thesis_perception/README.md).
   No iniciar otra cámara o adaptador de robot si ya hay una sesión activa.
2. Comprobar que no hay otro publicador en los tres tópicos de segmentación.
   Detener solo el prototipo anterior de esta visualización si corresponde.
3. En otra terminal con ROS y el workspace sourced:

```bash
tools/perception/experimental_arm_segmentation/run.sh
```

4. En otra terminal, abrir el perfil guardado:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d tools/perception/experimental_arm_segmentation/segmentation.rviz
```

El perfil utiliza `/codex/segmentacion/brazo_candidato`,
`/codex/segmentacion/margen_incierto` y
`/codex/segmentacion/resto_candidato`. La cámara y RViz pueden iniciarse según
el procedimiento del paquete; este ejecutable no los inicia. No usar la
herramienta con el brazo físico en movimiento sin seguir las condiciones de
operación y confirmaciones del proyecto.

## Verificación y restricciones

`python3 tools/perception/experimental_arm_segmentation/test_gpu.py` compara
un caso sintético contra SciPy y requiere GPU/sesión gráfica. Se comprobó la
versión preservada mediante reproducción del bag de la prueba estática en un
dominio ROS aislado y reloj simulado: para marcas comunes, 239 nubes de brazo,
279 inciertas y 279 de resto coincidieron byte por byte con la grabación del
prototipo original. El número de capturas difirió por el transporte y la
reproducción; esto no demuestra equivalencia para todas las escenas ni
exactitud física. El programa acepta parámetros ROS adicionales después de
`run.sh`; para reproducción aislada se utilizó `-p use_sim_time:=true`.

Este prototipo conserva las limitaciones originales: requiere transforms a la
marca de la captura, descarta muestras si no están disponibles, depende de un
muestreo de mallas y usa un proceso GPU auxiliar. No convierte la superposición
experimental en calibración. El resto candidato puede incluir brazo o excluir
objetos cercanos. Los rosbags `.db3` se mantienen locales y Git los ignora;
conservar una copia de seguridad separada con los `metadata.yaml`.
