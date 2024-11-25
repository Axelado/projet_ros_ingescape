import pygame
import time
import json

import sys
import ingescape as igs

def joystick():
    # Initialiser Pygame
    pygame.init()
    
    # Initialiser le joystick
    pygame.joystick.init()
    
    while pygame.joystick.get_count() == 0:
        print("Aucun joystick détecté.")
        time.sleep(0.5)
        
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick connecté : {joystick.get_name()}")

    try:
        while True:
            # Gérer les événements
            pygame.event.pump()
            
            # Lire les axes du joystick
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            
            # Lire les boutons du joystick
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            
            # Lire les hats (croix directionnelle, souvent)
            hats = [joystick.get_hat(i) for i in range(joystick.get_numhats())]
            
            # Transformer les entrées en chaîne de caractères
            joystick_state = f"Axes: {axes}, Buttons: {buttons}, Hats: {hats}"
            # print(joystick_state)
            
            # Obtenir l'horodatage actuel
            timestamp = time.time()
            sec = int(timestamp)
            nanosec = int((timestamp - sec) * 1e9)
            
            # Construire l'état sous forme de dictionnaire
            joystick_state = {
                "timestamp": {
                    "sec": sec,
                    "nanosec": nanosec
                },
                "axes": axes,
                "buttons": buttons,
                "hats": hats
            }
            
            # Convertir l'état en chaîne de caractères au format JSON
            joystick_state_json = json.dumps(joystick_state)
            print(joystick_state_json)
            igs.output_set_string("joystick_state", joystick_state_json)

            
            
            # Pause pour limiter la fréquence de mise à jour
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArrêt du programme.")
    
    finally:
        # Nettoyage
        joystick.quit()
        pygame.quit()    
        igs.stop()


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("usage: python3 main.py agent_name network_device port")
        devices = igs.net_devices_list()
        print("Please restart with one of these devices as network_device argument:")
        for device in devices:
            print(f" {device}")
        exit(0)
        
    igs.agent_set_name(sys.argv[1])
    igs.log_set_console(True)
    igs.log_set_file(True, None)
    igs.set_command_line(sys.executable + " " + " ".join(sys.argv))

    igs.debug(f"Ingescape version: {igs.version()} (protocol v{igs.protocol()})")

    igs.output_create("joystick_state", igs.STRING_T, None)

    igs.start_with_device(sys.argv[2], int(sys.argv[3]))
    
    joystick()
    
    igs.stop()
