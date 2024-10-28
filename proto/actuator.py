import socket
import numpy as np
from proto.ssl_simulation_robot_control_pb2 import RobotControl, MoveWheelVelocity, MoveGlobalVelocity, MoveLocalVelocity

def rotate_vector(v, theta):
    """
    Descrição:
            Função que recebe um vetor v e o rotaciona em theta graus
    Entradas:
            v:      Vetor numpy [1x2]
            theta:  Angulo de rotação do vetor
    """
    # Matriz de rotação
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))

    # Rotação do vetor
    v_rotated = np.matmul(R, v)

    return v_rotated



class Actuator():
    def __init__(self, ip:str='localhost', port:int=10000,team_port:int=10302, logger:bool=False) -> None:
        """
        Descrição:
                Classe para interação com um atuador em um sistema de controle ou automação.

        Entradas:
                ip:             Endereço IP para comunicação. Padrão é 'localhost'.
                port:           Porta de comunicação. Padrão é 10000.
                team_port:      Porta da equipe. Padrão é 10302.
                logger:         Flag que ativa o log de recebimento de mensagens no terminal. Por 
                                padrão se mantém desativado
        """
        # Newtork parameters
        self.ip = ip
        self.port = port
        self.team_port = team_port
        self.buffer_sice = 65536 # Parametro que define o tamanho da palavra binária a ser recebida da rede
 
        # Logger control
        self.logger = logger

        # Create socket
        self._create_socket()



    def _create_socket(self):
        '''
        Descrição:  
                Método responsável pela criação do socket de conexão com o servidor de visão
        '''
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.ip, self.port))
        self.socket.setblocking(False) 
        self.socket.settimeout(0.0)

    
    def send_socket(self, data):
        '''
        Descrição:  
                Método responsável pelo envio da mensagem para o simulador
        '''
        try:
            self.socket.sendto(data, (self.ip, self.team_port))
            if self.logger: print("[Actuator] Enviado!")

        except socket.error as e:
            if e.errno == socket.errno.EAGAIN:
                if self.logger:
                    print("[Actuator] Falha ao enviar. Socket bloqueado")
            else:
                print("[Actuator] Socket error:", e)



    def send_wheelVelocity_message(self, index, wheel_bl, wheel_br, wheel_fl, wheel_fr):
        '''
        Descrição:  
                Método responsável pelo envio das velocidades diretamente para as rodas do robô
        '''
        self.robot_id = index
        self.wheel_bl = wheel_bl
        self.wheel_br = wheel_br
        self.wheel_fl = wheel_fl
        self.wheel_fr = wheel_fr

        # Crie uma mensagem RobotControl
        robot_control = RobotControl()

        #Crie uma mensagem RobotCommand
        robot_command = robot_control.robot_commands.add()
        robot_command.id = self.robot_id

        #Crie uma mensagem MoveWheelVelocity
        move_command = MoveWheelVelocity()
        move_command.front_right = self.wheel_fr
        move_command.back_right = self.wheel_fl
        move_command.back_left = self.wheel_br
        move_command.front_left = self.wheel_bl

        # Atribua a mensagem MoveWheelVelocity ao campo move_command da mensagem RobotCommand
        robot_command.move_command.wheel_velocity.CopyFrom(move_command)

        self.send_socket(robot_control.SerializeToString())
        


    def send_globalVelocity_message(self, robot,velocity_x, velocity_y, angular):
        '''
        Descrição:  
                Método responsável pelo envio da velocidade global do robô
        '''

        # Módulo da velocidade linear
        mod_v = np.sqrt(velocity_x*velocity_x + velocity_y*velocity_y)
        
        # Correção da velocidade para os limites desejados
        if mod_v > robot.v_max:
            velocity_x = velocity_x * robot.v_max/mod_v
            velocity_y = velocity_y * robot.v_max/mod_v
        
        self.robot_id = robot.robot_id
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y
        self.angular = angular
        

        # Crie uma mensagem RobotControl
        robot_control = RobotControl()

        #Crie uma mensagem RobotCommand
        robot_command = robot_control.robot_commands.add()
        robot_command.id = self.robot_id

        #Crie uma mensagem MoveGlobalVelocity
        move_command = MoveGlobalVelocity()
        move_command.x = self.velocity_x
        move_command.y = self.velocity_y
        move_command.angular = self.angular

        # Atribua a mensagem MoveGlobalVelocity ao campo move_command da mensagem RobotCommand
        robot_command.move_command.global_velocity.CopyFrom(move_command)


        self.send_socket(robot_control.SerializeToString())

    def send_localVelocity_message(self, robot_id, vx_local, vy_local, angular):
        '''
        Descrição:  
                Método responsável pelo envio da velocidade local do robô
        '''
        wheel_radius = 0.09
        robot_radius = 0.027
        phi1 = 60 * np.pi/180
        phi2 = 135 * np.pi/180
        phi3 = 225 * np.pi/180
        phi4 = 300 * np.pi/180

        # Crie uma mensagem RobotControl
        robot_control = RobotControl()

        #Crie uma mensagem RobotCommand
        robot_command = robot_control.robot_commands.add()
        robot_command.id = robot_id

        # Transformação do vetor local de velocidades do robô para as rodas
        # Fonte: grSim/src/robot.cpp
        dw1 =  (1.0 / wheel_radius) * (( (robot_radius * angular) - (vx_local * np.sin(phi1)) + (vy_local * np.cos(phi1))) )
        dw2 =  (1.0 / wheel_radius) * (( (robot_radius * angular) - (vx_local * np.sin(phi2)) + (vy_local * np.cos(phi2))) )
        dw3 =  (1.0 / wheel_radius) * (( (robot_radius * angular) - (vx_local * np.sin(phi3)) + (vy_local * np.cos(phi3))) )
        dw4 =  (1.0 / wheel_radius) * (( (robot_radius * angular) - (vx_local * np.sin(phi4)) + (vy_local * np.cos(phi4))) )

        # Por algum motivo os motores precisam ir de 4 até 1... 
        # O simulador inverteu os motores
        self.send_wheelVelocity_message(robot_id, dw4, dw3, dw2, dw1)

        #self.send_socket(robot_control.SerializeToString())

    
    def send_wheel_from_global(self, robot, velocity_x, velocity_y, angular):
        '''
        Descrição:  
                Método responsável pelo envio da velocidade de cada roda do robô 
                em função da velocidade global (vx, vy, w) dele.
        '''

        # Módulo da velocidade linear
        mod_v = np.sqrt(velocity_x*velocity_x + velocity_y*velocity_y)
        
        # Correção da velocidade para os limites desejados
        if mod_v > robot.v_max:
            velocity_x = velocity_x * robot.v_max/mod_v
            velocity_y = velocity_y * robot.v_max/mod_v

        angle = robot.get_coordinates().rotation # Angulo do robô

        vector_vel = [velocity_x, velocity_y] # Vetor de velocidades
        vector_vel = np.array(vector_vel)

        # Transformação do vetor em global para o local do robô
        vector_vel_local = rotate_vector(vector_vel, - angle) 
        vx_local = vector_vel_local[0]
        vy_local = vector_vel_local[1]

        # Transformação do vetor local de velocidades do robô para as rodas
        # Fonte: grSim/src/robot.cpp
        dw1 =  (1.0 / robot.wheel_radius) * (( (robot.robot_radius * angular) - (vx_local * np.sin(robot.phi1)) + (vy_local * np.cos(robot.phi1))) )
        dw2 =  (1.0 / robot.wheel_radius) * (( (robot.robot_radius * angular) - (vx_local * np.sin(robot.phi2)) + (vy_local * np.cos(robot.phi2))) )
        dw3 =  (1.0 / robot.wheel_radius) * (( (robot.robot_radius * angular) - (vx_local * np.sin(robot.phi3)) + (vy_local * np.cos(robot.phi3))) )
        dw4 =  (1.0 / robot.wheel_radius) * (( (robot.robot_radius * angular) - (vx_local * np.sin(robot.phi4)) + (vy_local * np.cos(robot.phi4))) )

        # Por algum motivo os motores precisam ir de 4 até 1... 
        # O simulador inverteu os motores
        self.send_wheelVelocity_message(robot.robot_id, dw4, dw3, dw2, dw1)


if __name__ == '__main__':
    import time
    actuator = Actuator()

    while True:
        t1 = time.time()
        actuator.send_wheelVelocity_message(4,15,1,15,1)
        actuator.send_globalVelocity_message(2,5,10,15)
        actuator.send_localVelocity_message(3,5,10,15)
        t2 = time.time()

        if( (t2-t1) < 1/300 ):
            time.sleep(1/300 - (t2-t1))