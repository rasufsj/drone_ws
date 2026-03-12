#!/bin/bash
# takeoff.sh — versão MINIMALISTA e INFALÍVEL (MRS UAV ROS2 Jazzy)

echo "=== TAKEOFF MINIMALISTA (só o essencial) ==="

# Apenas espera 15 segundos (tempo suficiente para tudo subir)
sleep 15

# Arming + Offboard direto (os serviços JÁ existem nesse ponto)
echo "Arming o drone..."
ros2 service call /$UAV_NAME/hw_api/arming std_srvs/srv/SetBool "{data: true}"

sleep 1

echo "Entrando em OFFBOARD..."
ros2 service call /$UAV_NAME/hw_api/offboard std_srvs/srv/Trigger "{}"

echo "TAKEOFF CONCLUÍDO! Drone subindo..."