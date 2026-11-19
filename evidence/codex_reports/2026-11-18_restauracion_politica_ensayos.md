# Ensayos tras restauración selectiva de política

Se ejecutaron tres sesiones aisladas en dominio 96 y particiones independientes mediante `probe_profile_jog.py`, sin modificar las sesiones gráficas existentes. Cada sesión alcanzó READY y los extremos de las cápsulas coincidieron con TF con error máximo inferior a 1,4 × 10⁻¹⁶ m.

- Libre: desplazamiento articular 1,007 rad, 451 decisiones, velocidad máxima 0,0804 rad/s; hubo un STOP inicial de admisión.
- Esfera fija: desplazamiento 0,44366 rad; STOP con separación actual 0,04992 m y nominal 0,04573 m; ninguna reducción. Reproduce el comportamiento anterior, que no cumple todavía el objetivo de reducción progresiva.
- STOP inyectado y retirada: 79 STOP y 216 decisiones REDUCTION a escala 0,1; la separación creció hasta 0,03176 m durante la retirada observada.

Las trazas completas están en `2026-11-18_restauracion_politica_trazas/`. La sonda devuelve deliberadamente estado inconcluso para JACO2; estos datos son comprobaciones del comportamiento restaurado, no validación integral de seguridad ni del volumen futuro.
