# Importación CAD del gripper: vista provisional

Se conserva la entrega completa en `third_party/gripper_instrumentado_export/`
y el paquete ROS en `src/gripper_instrumentado_description/`. Se verificaron
los 78 archivos del manifiesto. El paquete compiló y se corrigió una variable
Xacro de la entrega que impedía expandir el modelo.

La vista `provisional_gripper_view.launch.py` muestra el STL del ensamble CAD
estático en UR17g, TM25S o P3020. El montaje es identidad provisional sobre el
frame terminal del modelo y no está calibrado; no representa un adaptador
validado. La vista no arranca Gazebo, controladores ni supervisión. La caja de
colisión cubre la malla en la pose exportada, sin validar la herramienta para
seguridad o carga. Los dedos no se mueven: faltan datos cinemáticos y reparar
siete restricciones Inventor. El stack normal de simulación permanece intacto.

Validación: compilación de los paquetes afectados, expansión URDF y 23 pruebas
de perfiles y vista CAD. No se validó visualmente una sesión gráfica ni la
integración Gazebo. El ZIP original se conserva porque el usuario condicionó
su traslado a la papelera a completar la integración.
