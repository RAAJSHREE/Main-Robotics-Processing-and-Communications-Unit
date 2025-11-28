# Main-Robotics-Processing-and-Communications-Unit

This project implements an integrated monitoring and communication architecture for a robotics processing and control system. It enables multi-protocol connectivity, real-time diagnostics, safety monitoring, and continuous cloud reporting.

The system includes multiple communication interfaces (CANOpen, Ethernet, LVDS/Optical) and ensures safe, reliable operation through sensor health checks, watchdogs, and heartbeat mechanisms.

**System Architecture**
Refer architecture_diagram.drawio.png
https://github.com/RAAJSHREE/Main-Robotics-Processing-and-Communications-Unit/blob/main/architecture_diagram.drawio.png


**Features**

Multi-interface communication: CANOpen / Ethernet / LVDS / Optical

Linux-based main processing unit

Real-time health monitoring & diagnostics

Safety Layer Manager (SLM) integration

Heartbeat monitoring & logs management

Live map and connectivity status 

Cloud connectivity for continuous data streaming

Alerting with timestamps and operator notifications

Modular, scalable architecture

**Files Included**

robot_agent.py – complete script of robot control

architecture.png – Architecture diagram

README.md – Project documentation

**Setup**
1.Using WSL
docker compose up --build
 
2.Using Python
    Pip install -r requirements.txt
    python robot_agent.py




