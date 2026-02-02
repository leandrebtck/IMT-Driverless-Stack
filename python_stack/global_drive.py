import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from std_srvs.srv import Empty 
from pynput import keyboard
import threading

class GlobalPilot(Node):
    def __init__(self):
        super().__init__('fsds_global_pilot')
        
        # Topic de commande
        self.pub = self.create_publisher(ControlCommand, '/control_command', 10)
        
        # Client Reset
        self.reset_client = self.create_client(Empty, '/reset') 
        
        # Variables d'état
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        
        # Timer : Envoi des commandes 10 fois par seconde (10Hz)
        self.timer = self.create_timer(0.1, self.publish_cmd)
        
        print("🚗 PILOTE MANUEL - MODE STABLE")
        print("------------------------------")
        print("   [Z] AVANCER (Gaz)")
        print("   [S] FREINER (Stop)")
        print("   [Q] GAUCHE")
        print("   [D] DROITE")
        print("   [R] RESET (Remise sur la piste)")
        print("------------------------------")

    def publish_cmd(self):
        cmd = ControlCommand()
        # On s'assure que les types sont float
        cmd.throttle = float(self.throttle)
        cmd.brake = float(self.brake) 
        cmd.steering = float(self.steering)
        
        self.pub.publish(cmd)
        
        # Debug optionnel pour voir ce qu'on envoie
        # print(f"Gaz: {cmd.throttle:.1f} | Frein: {cmd.brake:.1f} | Volant: {cmd.steering:.1f}")

    def update(self, key, pressed):
        try:
            k = key.char
        except:
            k = key.name

        # --- LOGIQUE : APPUYER SUR UNE TOUCHE ---
        if pressed:
            if k == 'z': 
                self.throttle = 0.4  # Vitesse constante modérée
                self.brake = 0.0     # On lâche le frein
                
            elif k == 's': 
                self.throttle = 0.0  # On coupe les gaz
                self.brake = 1.0     # ON ÉCRASE LE FREIN (MAX)
            
            elif k == 'q': 
                self.steering = -0.5 # Gauche
            elif k == 'd': 
                self.steering = 0.5  # Droite

            elif k == 'r':
                self.reset_client.call_async(Empty.Request())
                self.throttle, self.brake, self.steering = 0.0, 1.0, 0.0
                print(" RESET !")

            elif k == 'esc':
                # Sécurité : on coupe tout
                self.throttle = 0.0
                self.brake = 1.0

        # --- LOGIQUE : RELÂCHER LA TOUCHE ---
        else:
            if k == 'z': 
                self.throttle = 0.0 # Roue libre
            
            if k == 's': 
                self.brake = 0.0    # On relâche le frein
            
            if k in ['q', 'd']: 
                self.steering = 0.0 # Retour volant centre

def start_keyboard_listener(node):
    # Gestionnaire de clavier non-bloquant
    with keyboard.Listener(
        on_press=lambda k: node.update(k, True),
        on_release=lambda k: node.update(k, False)
    ) as listener:
        listener.join()

def main():
    rclpy.init()
    node = GlobalPilot()
    
    # Le clavier tourne dans un thread séparé pour ne pas bloquer ROS
    kb_thread = threading.Thread(target=start_keyboard_listener, args=(node,))
    kb_thread.daemon = True
    kb_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Arrêt propre
        stop = ControlCommand()
        stop.brake = 1.0
        node.pub.publish(stop)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()