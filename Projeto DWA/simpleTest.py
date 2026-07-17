import sim
import time

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Conectado à Base Funcional')

    _, motorEsq = sim.simxGetObjectHandle(clientID, 'MOTOR_ESQUERDO', sim.simx_opmode_blocking)
    _, motorDir = sim.simxGetObjectHandle(clientID, 'MOTOR_DIREITO', sim.simx_opmode_blocking)
    
    sensores = ['SENSOR_ESQUERDO', 'SENSOR_DIAG_ESQUERDO', 'SENSOR_MEIO', 'SENSOR_DIAG_DIREITO', 'SENSOR_DIREITO']
    s_handles = []
    for s in sensores:
        _, h = sim.simxGetObjectHandle(clientID, s, sim.simx_opmode_blocking)
        s_handles.append(h)
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    time.sleep(1) 

    vel_linear = 2.0
    dist_segura = 0.5
    memoria_giro = 0
    direcao = 1 

    try:
        while True:
            leituras = []
            for h in s_handles:
                res, detectionState, distPoint, _, _ = sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_buffer)
                
                dist = 999
                if detectionState:
                    dist = (distPoint[0]**2 + distPoint[1]**2 + distPoint[2]**2)**0.5
                leituras.append(dist)

            perigo_frontal = leituras[1] < dist_segura or leituras[2] < dist_segura or leituras[3] < dist_segura
            perigo_lateral = leituras[0] < dist_segura or leituras[4] < dist_segura

            if perigo_frontal or perigo_lateral or memoria_giro > 0:
                if perigo_frontal or perigo_lateral:
                    memoria_giro = 10 
                    direcao = 1 if (leituras[0] < dist_segura or leituras[1] < dist_segura) else -1
                
                sim.simxSetJointTargetVelocity(clientID, motorEsq, vel_linear * direcao, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, motorDir, -vel_linear * direcao, sim.simx_opmode_oneshot)
                memoria_giro -= 1
            else:
                sim.simxSetJointTargetVelocity(clientID, motorEsq, vel_linear, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, motorDir, vel_linear, sim.simx_opmode_oneshot)

            time.sleep(0.02)

    except KeyboardInterrupt:
        sim.simxSetJointTargetVelocity(clientID, motorEsq, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, motorDir, 0, sim.simx_opmode_blocking)
        sim.simxFinish(clientID)