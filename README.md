# Final-Challenge

![Tec](https://brandemia.org/sites/default/files/sites/default/files/tec_monterrey_nuevo_logo.png)

Este proyecto es una colaboración entre los siguientes estudiantes de la Universidad:

- **Salvador Santana Blanco**            A01703523
- **Anatanael Jesus Miranda Faustino**  A01769232
- **Diego Díaz Ayala**            A01770236
- **Carlos Eduardo Ortega**        A01707480


*Profesor: Juan Manuel Ledesma Rangel*
*Profesor: Jesús Arturo Escobedo Cabello*
*Profesor: Pablo Estéfano Arroyo Garrido*
*Profesor: Juan Manuel Ledesma Rangel*


# Proyecto de Localización y Navegación Autónoma en ROS

Este proyecto implementa el filtro de Kalman y un mensaje de odometría para estimar la posición de un vehículo autónomo en el entorno de ROS (Robot Operating System). Además, se incluyen implementaciones de los algoritmos de navegación Bug Two y Bug Zero para permitir que el vehículo navegue hacia una meta mientras evita obstáculos en su camino.

## Contenidos del Proyecto

### Archivos y Directorios

- **CMakeLists.txt**: Archivo de configuración de CMake para el proyecto.
- **include/final**: Directorio para archivos de encabezado (headers) si es necesario.
- **launch**: Directorio que contiene los archivos de lanzamiento.
  - `all.launch`: Archivo de lanzamiento principal que inicia todos los nodos necesarios para la localización y navegación.
- **package.xml**: Archivo de configuración del paquete ROS.
- **scripts**: Directorio que contiene los scripts de Python para los algoritmos y otros módulos auxiliares.
  - `bug_two.py`: Implementación del algoritmo Bug Two para la navegación.
  - `bug_zero.py`: Implementación del algoritmo Bug Zero para la navegación.
  - `conditions.py`: Módulo con condiciones auxiliares utilizadas por los algoritmos de navegación.
  - `localisation.py`: Implementación del sistema de localización utilizando el filtro de Kalman y el mensaje de odometría.
  - `pid.py`: Implementación de un controlador proporcional-integral-derivativo (PID) para el control de movimiento.
  - `wf.py`: Módulo para la funcionalidad "Wall-Following".
  - Archivos bytecode compilados de Python en el directorio `__pycache__`.

### Comandos de Ejecución

Para ejecutar el proyecto, se utilizan los siguientes comandos:

```bash
roslaunch all.launch
rosrun localisation.py
rosrun bug_two.py 
rosrun bug_zero.py
