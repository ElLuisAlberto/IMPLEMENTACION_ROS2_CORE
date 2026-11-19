# Datos pendientes para la cinemática ROS 2

La pose exportada se ha documentado, pero esta entrega no declara límites, ejes, sentido positivo del servo, apertura, TCP ni muestras de recorrido como valores validados.

La razón es verificable en `inventor_constraints_health.csv`: siete restricciones del ensamblaje fuente tienen estado `driver_lost` (referencia conductora desconectada). Afectan el acoplamiento leva-servo, insertos asociados a la leva, las bielas y los eslabones interiores:

- `Coincidencia:35`: `LEVA_ACCIONAMIENTO:1` / `FEETECH_ST-3215-C018:1`
- `Nivelación:8` y `Nivelación:9`: insertos / leva
- `Nivelación:20` y `Nivelación:21`: bielas / insertos
- `Coincidencia:87` y `Coincidencia:89`: bielas / eslabones interiores

Antes de generar el `xacro`, repare o reemplace esas relaciones, defina los ejes en geometría nominal y mida el recorrido en al menos quince poses. Las filas `CAD_SAMPLE_01` a `CAD_SAMPLE_15` de `kinematics/servo_to_opening.csv` son marcadores de adquisición, no mediciones.
