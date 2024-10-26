import time
import socket
import threading
from proto.ssl_simulation_robot_control_pb2 import RobotControl

RECEIVER_FPS = 300  # Taxa de aquisição da rede dos pacotes do software
CONTROL_FPS = 60  # Taxa de envio para o STM (Pode alterar aqui se necessário)

# ---------------------------------------------------------------------------------------------
#    DEFINIÇÃO DE CLASSES
# ---------------------------------------------------------------------------------------------

class RepeatTimer(threading.Timer):
    """
    Descrição:
        Classe herdada de Timer para execução paralela da thread de recebimento das velocidades
    """
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

class RobotVelocity:
    """
    Descrição:
        Classe para armazenar as velocidades de cada robô
    Entradas:
        id_robot:   Robô que corresponde às velocidades do objeto (0 a 2)
    """
    def __init__(self, id_robot):
        self.id_robot = id_robot  # id do robô
        # Velocidades angulares
        self.wheel_velocity_front_right = 0 
        self.wheel_velocity_back_right = 0
        self.wheel_velocity_back_left = 0
        self.wheel_velocity_front_left = 0

        self.cont_not_message = 0
        self.treshold_message = 2*RECEIVER_FPS

class Receiver():
    def __init__(self, ip: str = 'localhost', port: int = 10302, logger: bool = False):
        """
        Descrição:
            Classe para recepção de mensagens serializadas usando Google Protobuf.
        
        Entradas:
            ip:       Endereço IP para escuta. Padrão é 'localhost'.
            port:     Porta de escuta. Padrão é 10302.
            logger:   Flag que ativa o log de recebimento de mensagens no terminal.
        """
        # Parâmetros de rede
        self.ip = ip
        self.port = port
        self.buffer_size = 65536  # Tamanho máximo do buffer para receber mensagens

        # Controle de log
        self.logger = logger

        # Robôs a serem controlados
        self.robot0 = RobotVelocity(0)
        self.robot1 = RobotVelocity(1)
        self.robot2 = RobotVelocity(2)
        self.robots = [self.robot0, self.robot1, self.robot2]

        # Criar socket
        self._create_socket()

    def _create_socket(self):
        """
        Descrição:
            Cria o socket UDP e configura-o para ser não-bloqueante.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.ip, self.port))
        self.socket.setblocking(False)
        self.socket.settimeout(0.0)  # Não bloqueia

    def receive_socket(self):
        """
        Descrição:
            Método responsável por receber a mensagem serializada e desserializá-la usando o Protobuf.
        
        Retorna:
            Instância da classe Protobuf desserializada ou None se não receber nada.
        """
        try:
            data, _ = self.socket.recvfrom(self.buffer_size)
            if self.logger:
                print("[Receiver] Mensagem recebida")

            # Desserializar a mensagem usando a classe Protobuf RobotControl
            message = RobotControl()
            message.ParseFromString(data)
            self.decode_message(message)

        except socket.error as e:
            if e.errno == socket.errno.EAGAIN:
                # Nenhuma mensagem disponível no momento
                # Se ficar muito tempo sem receber mensagens, a velocidade do robô vai a zero
                for robot in self.robots:
                    if robot.cont_not_message > robot.treshold_message:
                        robot.wheel_velocity_front_right = 0
                        robot.wheel_velocity_back_right = 0
                        robot.wheel_velocity_back_left = 0
                        robot.wheel_velocity_front_left = 0
                    else:
                        robot.cont_not_message += 1
                
                return None
            else:
                print("[Receiver] Erro de socket:", e)
                return None
            
    def decode_message(self, message):
        id_robot = message.robot_commands[0].id
        wheel_velocity_front_right = message.robot_commands[0].move_command.wheel_velocity.front_right
        wheel_velocity_back_right = message.robot_commands[0].move_command.wheel_velocity.back_right
        wheel_velocity_back_left = message.robot_commands[0].move_command.wheel_velocity.back_left
        wheel_velocity_front_left = message.robot_commands[0].move_command.wheel_velocity.front_left
        self.robots[id_robot].wheel_velocity_front_right = wheel_velocity_front_right
        self.robots[id_robot].wheel_velocity_back_right = wheel_velocity_back_right
        self.robots[id_robot].wheel_velocity_back_left = wheel_velocity_back_left
        self.robots[id_robot].wheel_velocity_front_left = wheel_velocity_front_left
        self.robots[id_robot].cont_not_message = 0

            
    def start_thread(self):
        """
        Descrição:
            Função que inicia a thread da visão
        """
        self.vision_thread = RepeatTimer((1 / RECEIVER_FPS), self.receive_socket)
        self.vision_thread.start()

# ---------------------------------------------------------------------------------------------
#   INICIO DO CÓDIGO PRINCIPAL
# ---------------------------------------------------------------------------------------------

# Inicio do recebimento das mensagens via socket
receiver = Receiver(port=10330, logger=False)
receiver.start_thread()

while True:
    t1 = time.time()
    # Acesso das variáveis obtidas pela rede em cada um dos robôs [0, 1 e 2]
    for robot in receiver.robots:
        print("Robô ", robot.id_robot)
        print("Frente direita: ", robot.wheel_velocity_front_right)
        print("Frente esquerda: ", robot.wheel_velocity_front_left)
        print("Trás direita: ", robot.wheel_velocity_back_right)
        print("Trás esquerda: ", robot.wheel_velocity_back_left)

    # Caso a interface com o teclado não esteja pronta ainda, descomente as linhas abaixo
    # Elas possuem casos padrão para testes básicos de validação.

    # Robô 0 a 0.5m/s pra frente - descomentar as próximas 5 linhas
    # robot0 = receiver.robots[0]
    # robot0.wheel_velocity_front_right = 3.33333
    # robot0.wheel_velocity_front_left = 3.33333
    # robot0.wheel_velocity_back_right = 3.33333
    # robot0.wheel_velocity_back_left = 3.33333

    # Robô 1 a 0.5m/s pra cima - descomentar as próximas 5 linhas
    # robot1 = receiver.robots[1]
    # robot1.wheel_velocity_front_right = 9.25926
    # robot1.wheel_velocity_front_left = 9.259256
    # robot1.wheel_velocity_back_right = -13.09457
    # robot1.wheel_velocity_back_left = -13.09457

    # Robô 2 a 1 rad/s (apenas girando) - descomentar as próximas 5 linhas
    # robot2 = receiver.robots[2]
    # robot2.wheel_velocity_front_right = -16.03751
    # robot2.wheel_velocity_front_left = 16.03751
    # robot2.wheel_velocity_back_right = -13.09457
    # robot2.wheel_velocity_back_left = -13.09457


    # ---------------------------------------------------------------------------------------------
    #   ESCREVA SEU CÓDIGO AQUI, MESTRES DOS 10 PINOS
    # ---------------------------------------------------------------------------------------------

    t2 = time.time()

    if (1/CONTROL_FPS - (t2-t1) > 0):
        time.sleep(1/CONTROL_FPS - (t2-t1))

        