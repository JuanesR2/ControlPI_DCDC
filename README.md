# Controlador PI para Convertidor DC-DC

Este script implementa y simula un **controlador PI** aplicado a un sistema DC-DC modelado con ecuaciones diferenciales. Utiliza integración numérica con `solve_ivp` de SciPy y visualiza la evolución de los estados en el tiempo.

Juan Esteban Rodríguez Villada
---

## 📌 Modelo del Sistema

Se parte de un sistema representado en forma de espacio de estados:

- Matriz **A**: representa la dinámica natural del sistema.
- Matriz **B**: representa la entrada de control.
- Vector **d**: perturbación o entrada constante.
- **u_ref**: valor de referencia de la entrada de control.
- **x_ref**: punto de equilibrio calculado resolviendo:

\[
x_{ref} = -(A + u_{ref} B)^{-1} d
\]

---

## 🎯 Controlador PI

El controlador utiliza dos ganancias:
- **kp**: Ganancia proporcional
- **ki**: Ganancia integral

Se definen dos funciones:

- `y_d(x)`: calcula una función de error ponderada.
- `u_control(x_e)`: aplica la ley de control PI sobre el error extendido.

La dinámica extendida se define como:

```python
def convertidor(t, u):
    x_e = u
    du1_2 = A @ x_e[:2] + u_control(x_e) * B @ x_e[:2] + d
    du3 = y_d(x_e[:2])
    return np.concatenate([du1_2, [du3]])
