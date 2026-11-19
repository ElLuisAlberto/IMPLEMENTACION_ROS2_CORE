# Pruebas de la célula de cuatro robots

Resultado: **PASS**, ROS 2 Humble / Gazebo Fortress, simulación exclusivamente.

| Verificación | Resultado |
| --- | --- |
| Compilación seleccionada | 6 paquetes aprobados |
| Pruebas paquete nuevo | 74 aprobadas, 0 fallos |
| Regresión funcional thesis_core | 173 aprobadas |
| Regresión funcional thesis_simulation | 41 aprobadas |
| Célula JACO2 + UR17g + TM25S + P3020 | PASS |
| Controladores aislados | 8 activos |
| J1 +0.02 rad y retorno, cada robot | Error <0.008 rad |
| Inserción de obstáculo durante movimiento | Cancelación y hold en los cuatro |
| Deriva medida en hold | Aproximadamente 0.000001 rad |
| Obstáculo en contacto / obstáculo caducado | STOP / rechazo en los cuatro |
| Cuatro órdenes simultáneas +0.01 rad | Error <0.004 rad, sin STOP inesperado |
| Herramienta de comandos sobre P3020 vivo | Candidato aceptado en ALLOW |

Evidencia final de la célula:
[all_result.json](2026-11-07_multi_robot_cell_final/all_result.json),
[all_launch.log](2026-11-07_multi_robot_cell_final/all_launch.log),
[unit_tests.xml](2026-11-07_multi_robot_cell_final/unit_tests.xml).

Evidencia de la herramienta:
[p3020_result.json](2026-11-07_multi_robot_operator_cli_v2/p3020_result.json).

Los intentos fallidos anteriores se conservan para trazabilidad y están
explicados en el informe técnico. No se instalaron paquetes con sudo ni se
habilitó hardware físico. El supervisor nuevo sigue siendo experimental:
no están validados autocolisión, retiming de REDUCTION ni frenado dinámico;
no incluye la GUI original ni MoveIt. Estas pruebas no acreditan seguridad
certificada ni equivalencia completa con el supervisor JACO2 original.
