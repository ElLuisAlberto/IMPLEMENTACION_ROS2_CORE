# Guía para trabajar en este proyecto

## Contexto y autonomía

- Antes de cambiar código, inspecciona el contexto relevante: arquitectura y documentación relacionadas, flujo completo afectado, implementaciones existentes y pruebas. Sigue las llamadas entre paquetes cuando el comportamiento cruce sus límites; no te limites al archivo señalado si la causa está en otra parte.
- Basa el diagnóstico en el código y en evidencia reproducible. Corrige la causa raíz con los cambios necesarios y coherentes, siguiendo los patrones del proyecto. Puedes editar, compilar y probar sin pedir autorización para cada paso.
- Amplía el alcance cuando sea necesario para que la solución quede completa y consistente; evita cambios ajenos al problema. Si hay varias interpretaciones con consecuencias importantes, pregunta antes de elegir.
- Revisa los cambios existentes antes de editar. Consérvalos e intégralos; no descartes ni sobrescribas trabajo ajeno.
- Trabaja primero en simulación. No actives salida ni ordenes movimiento al JACO2 físico sin confirmar que el usuario es operador autorizado, el área está despejada y la parada de emergencia es accesible.
- Antes de la primera prueba física, verifica conexión, diagnóstico, armado y disponibilidad sin mover el brazo. Para cada etapa que pueda moverlo, comienza con desplazamientos mínimos y velocidad reducida, y solicita confirmación del operador.
- No uses `sudo` ni instales paquetes sin autorización. Si una aplicación gráfica falla por el entorno de la terminal integrada, diagnostica primero sin modificar el sistema; pide usar una terminal nativa solo si es imprescindible.

## Validación y comunicación

- Tras los cambios, compila solo los paquetes afectados y ejecuta las pruebas pertinentes. Comprueba el resultado real y comunica con claridad cualquier fallo o validación que no haya sido posible.
- No crees informes o archivos de planificación rutinarios. Al terminar, da un resumen breve de la causa, los cambios y la validación; crea informes solo si el usuario los pide o la tarea lo requiere.
- Si se solicita un informe, no lo guardes en la raíz: diagnósticos en `docs/codex_reports/` y resultados de pruebas en `evidence/codex_reports/`. Crea la carpeta correspondiente si hace falta y no sobrescribas informes existentes.

## Observación de ROS 2

- Para consultar nodos o tópicos con el CLI `ros2`, usa `--no-daemon --spin-time 2` para evitar el daemon de `ros2cli`.
- Ejecuta cada cliente de observación por separado y espera a que termine antes de iniciar el siguiente.
- `tf2_echo` no acepta esas opciones; si hay que detenerlo, detén solo ese cliente, nunca el proceso de lanzamiento.
