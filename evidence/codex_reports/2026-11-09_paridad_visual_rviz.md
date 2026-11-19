# Paridad visual RViz de perfiles colaborativos

Fecha: 2026-11-09

## Alcance

- Igualar la paleta y transparencia de las capsulas por perfil con el
  visualizador historico del JACO2.
- Evitar que `current_capsules` se renderice simultaneamente desde los
  displays `SafetyCapsules` y `HorizonVolume`.
- Hacer determinista la recepcion tardia de `robot_description` en RViz.
- No modificar umbrales, geometria, cinematica, control ni hardware.

## Cambios

- Estado `ALLOW`: RGBA `(0.05, 0.80, 0.35, 0.38)`.
- Estados `WARNING`, `REDUCTION` y `STOP`: paleta historica exacta.
- `RobotModel/Description Topic/Durability Policy`: `Transient Local`.
- `HorizonVolume/current_capsules`: desactivado; las predicciones, trazas,
  posturas y etiquetas futuras permanecen activas.

## Pruebas

```text
python3 -m pytest -q \
  src/thesis_core/test/test_profile_proximity_monitor.py \
  src/thesis_multi_robot/test/test_rviz_profile.py

12 passed
```

La compilacion colcon y la validacion grafica se ejecutan durante la
integracion final, despues de combinar las correcciones paralelas de URI y
materiales para evitar escrituras concurrentes en `build/` e `install/`.
