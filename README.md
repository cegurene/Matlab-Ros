# Matlab-Ros 🤖📡

[![Made with MATLAB](https://img.shields.io/badge/Made%20with-MATLAB-orange?logo=matlab)](https://www.mathworks.com/products/matlab.html)
[![ROS](https://img.shields.io/badge/ROS-Robot%20Operating%20System-blue?logo=ros)](https://www.ros.org/)

**Matlab-Ros** es un repositorio orientado a la integración entre **MATLAB** y **ROS (Robot Operating System)**.
Aquí encontrarás ejemplos y módulos para trabajar con percepción y control de robots, utilizando la potencia de MATLAB dentro de un entorno distribuido ROS.
Se usa también ROS2.

---

## 🚀 Resumen del proyecto

* Conecta **MATLAB** con **ROS** para que algoritmos escritos en MATLAB puedan publicar, suscribirse y procesar datos en tiempo real.
* Incluye ejemplos de **percepción** y **control** aplicados a sistemas robóticos.
* Código 100 % MATLAB, enfocado en **prototipado rápido** y **experimentación**.
* Organización modular: cada carpeta corresponde a un conjunto de prácticas o algoritmos específicos.

---

## 🔧 Funcionalidades principales

* 📡 **Comunicación ROS**: suscripción/publicación de tópicos directamente desde MATLAB.
* 👀 **Percepción**: procesamiento de datos sensoriales y señales para obtener información útil.
* 🎮 **Control**: algoritmos de control que generan acciones de movimiento y decisiones para el robot.
* 🧩 **Modularidad**: se puede usar cada parte por separado o integrarlas para cerrar el ciclo percepción-decisión-acción.

---

## 📂 Estructura del repositorio

 * `Percepcion y Control/` → Ejemplos relacionados con percepción y control. Lectura de sensores, mapeado y navegación usando controladores.
	 * `P1/`→ Lectura de sensores de distancia y odometría. Mover al robot por un entorno controlado.
	 * `P2/`→ Uso de distintos tipos de controladores para mover al robot. Seguimiento de paredes a una distancia fija.
	 * `TFA/`→ Exploración y mapeado de un entorno desconocido guiando al robot hacia zonas inexploradas. Creación del mapa mediante uso de sensores láser. Evasión de obstáculos.

 * `Sistemas_Control/` → Implementaciones de sistemas de control clásicos y avanzados en MATLAB. Lectura de sensores, mapeado y navegación autónoma.  Se encuentran todos los archivos. 
	 * Creación de mapas (mapeado con posiciones conocidas, SLAM).
	 * Localización usando AMCL. Localizar un robot utilizando datos del láser y la odometría sobre un mapa conocido a priori.
	 * Planificación local. Evitación de obstáculos usando VFH.
	 * Planificación global. Navegación de origen a destino con PurePursuit y generar trayectoria con PRM.
