# Sistema CAD completo: vista provisional en tres brazos

La entrega CAD completa está en `third_party/sistema_gripper_export/` y el
paquete ROS en `src/sistema_gripper_instrumentado_description/`. Se verificaron
20 hashes y la integridad del ZIP. Se corrigió la escala Xacro del paquete y
se compiló. El ZIP original se trasladó a la papelera del usuario.

Las vistas provisionales de UR17g, TM25S y P3020 usan el conjunto estático
completo, incluidos gripper, cámara y electrónica. El montaje es una estimación
visual de 90 grados y 63 mm, deducida de la malla, no un adaptador validado.
No se modificó la ruta supervisada ni el JACO2; el módulo no abre/cierra y no
se simulan sensores. La caja envolvente corresponde solo a la pose estática.

Se compilaron paquetes, pasaron 23 pruebas y se creó la entidad UR17g en
Gazebo headless. Se abrió RViz UR17g tras limpiar el entorno heredado de Snap.
No se ha validado visualmente la alineación ni el comportamiento de seguridad.
