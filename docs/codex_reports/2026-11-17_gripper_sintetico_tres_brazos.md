# Gripper sintético para ensayos de seguridad multi-brazo

Se añadió `synthetic_gripper` a los tres JACO2 de Gazebo del escenario híbrido.
Es un modelo geométrico de ensayo con dos dedos fijos en apertura máxima; no
representa el CAD PincOpen ni implementa apertura, cierre, agarre o carga.
El perfil de seguridad añade a las seis cápsulas del brazo una esfera que
contiene la envolvente sintética, y rechaza estados de carga sin perfil.
`arm_01` conserva la interfaz sintética anterior. No se habilita hardware.

Validación: compilación de `thesis_description`, `thesis_core` y
`thesis_simulation`; 13 pruebas de perfil y 27 pruebas de escenario,
alineación URDF y contrato geométrico; expansión Xacro y `check_urdf`.
Un arranque headless aislado creó las tres entidades y alcanzó `READY`; se
detuvo deliberadamente mediante timeout. Esto no demuestra la reacción STOP
ante proximidad ni la actuación de los dedos, que requieren ensayos específicos.
