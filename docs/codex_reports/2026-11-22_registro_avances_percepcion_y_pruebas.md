# Registro de avances: percepción D435i y filtro del JACO2

Fecha: 22 de noviembre de 2026. Actualización posterior: el prototipo se preservó con compilación y arranque separados en [`tools/perception/experimental_arm_segmentation/`](../../tools/perception/experimental_arm_segmentation/README.md); las referencias a `/tmp` abajo describen el proceso original y la situación al elaborar este índice. Este índice identifica los avances de la sesión, sus evidencias y lo que todavía debe hacerse para poder recuperarlos. No representa una validación del filtro para seguridad física.

## Avances registrados

| Avance | Estado y ubicación |
|---|---|
| Configuración de la D435i y visualización RGB-D | El arranque y sus parámetros están descritos en [`thesis_perception/README.md`](../../thesis_perception/README.md#standalone-mode). |
| Superposición visual cámara–brazo | El nodo, parámetros y perfil RViz experimental están en `thesis_perception/`. El ajuste corresponde al montaje observado y **no es una calibración extrínseca validada**. |
| Separación del brazo y margen incierto | Se ejecutó un prototipo experimental C++/GPU. Fuentes, binarios, muestras de malla y configuraciones se copiaron a [`evidence/codex_reports/2026-11-22_auditoria_tecnica_d435i_jaco2/`](../../evidence/codex_reports/2026-11-22_auditoria_tecnica_d435i_jaco2/). El proceso activo usa ejecutables y datos en `/tmp`; la copia archivada conserva rutas propias de esa sesión y **no constituye un arranque portable**. |
| Escenario de obstáculo | El README principal documenta el escenario virtual PARK y la recuperación de fuente ausente. PARK no detecta obstáculos físicos. |
| ChArUco | El tablero, las condiciones de uso y su función en los experimentos constan en la auditoría técnica. El tablero no proporciona por sí mismo una extrínseca cámara–robot validada. |
| Prueba estática | [Método y resultados](../../evidence/codex_reports/2026-11-22_prueba_filtro_estatico_resultados/RESULTADOS.md) y [rosbag](../../evidence/codex_reports/2026-11-22_prueba_filtro_estatico/metadata.yaml). |
| Primera prueba dinámica J1 | [Método y resultados](../../evidence/codex_reports/2026-11-22_prueba_filtro_movimiento_resultados/RESULTADOS.md) y [rosbag](../../evidence/codex_reports/2026-11-22_filtro_j1_movimiento_10grados/metadata.yaml). |
| Ensayo rápido simulado y pérdida de TF | [Resultado y diagnóstico](../../evidence/codex_reports/2026-11-22_prueba_j1_rapida_simulacion/RESULTADOS.md). La trayectoria rápida se probó en simulación; no se ejecutó físicamente en esa etapa. |
| Movimientos realizados por el operador | [Método y resultados](../../evidence/codex_reports/2026-11-22_movimiento_operador_resultados/RESULTADOS.md), [análisis detallado](../../evidence/codex_reports/2026-11-22_movimiento_operador_resultados/resultados_filtro.json) y [rosbag](../../evidence/codex_reports/2026-11-22_movimiento_operador/metadata.yaml). |
| Auditoría y cronología | [Auditoría técnica integral](2026-11-22_auditoria_tecnica_integral_d435i_jaco2.md) e [histórico de segmentación](2026-11-22_auditoria_historico_segmentacion_d435i_jaco2.md). |

Las pruebas comparan la clasificación con una referencia CPU basada en las mismas mallas y umbrales y verifican conservación de registros en capturas seleccionadas. No demuestran por sí solas precisión física frente a etiquetas independientes ni detección fiable de obstáculos reales.

## Recuperación y respaldo

1. **Código y documentación:** a la fecha del registro hay cambios y archivos nuevos sin confirmar en Git. Mantenerlos en el árbol de trabajo no equivale a haber creado un commit ni una copia externa. Revisar `git status` antes de hacer cualquier operación de versionado; no descartar cambios existentes.
2. **Grabaciones:** `.gitignore` excluye los archivos `*.db3` y `*.mcap`. Un commit o clon no preserva los rosbags. Se requiere respaldar por separado cada carpeta de bag completa, incluido `metadata.yaml` y el archivo de datos, y comprobar que la copia pueda leerse. El bag de movimientos del operador ocupa aproximadamente 14,5 GiB.
3. **Prototipo temporal:** mientras exista `/tmp`, el proceso puede seguir funcionando en la sesión actual, pero esa ubicación puede limpiarse. Las copias en la auditoría preservan artefactos, no garantizan que los binarios funcionen en otro entorno. Para un arranque repetible hay que integrar el código y dependencias en el paquete, sustituir rutas temporales, documentar parámetros y validar compilación y resultados antes de utilizarlo.
4. **Integridad:** hay manifiestos SHA-256 en varios directorios de evidencia; un hash permite comprobar coincidencia de bytes, no prueba validez experimental. No se ha realizado ni comprobado aquí una copia externa de respaldo.
5. **Seguridad:** la superposición, el filtro y el ajuste de cámara siguen siendo experimentales y están aislados de la entrada preventiva. La esfera virtual no observa personas ni obstáculos físicos. La interrupción de TF registrada en la prueba rápida simulada debe investigarse antes de nuevas pruebas físicas aceleradas.

Este registro no modifica configuraciones activas, no arma ni mueve el brazo y no inicia procesos.
