import tkinter as tk
from tkinter import messagebox

import sys
import ingescape as igs


def root_window():
    def on_button_click():
        selected_option = dropdown_var.get()
        room_number = options.index(selected_option)+1
        igs.output_set_int("room_number", room_number)
        messagebox.showinfo("Envoyé", f"Le robot va à la salle : {room_number}")
        
    # Création de la fenêtre principale
    root = tk.Tk()
    root.title("GO TO ROOM")

    # Fixer la taille de la fenêtre à 400x300 pixels
    root.geometry("220x100")

    # Liste des options pour la liste déroulante
    options = ["Salle 1", "Salle 2", "Salle 3", "Salle 4"]

    # Création de la variable associée à la liste déroulante
    dropdown_var = tk.StringVar()
    dropdown_var.set(options[0])  # Valeur par défaut

    # Création de la liste déroulante
    dropdown = tk.OptionMenu(root, dropdown_var, *options)
    dropdown.pack(pady=10)

    # Création du bouton
    button = tk.Button(root, text="GO TO", command=on_button_click)
    button.pack(pady=10)


    # Boucle principale
    root.mainloop()
    

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

    igs.output_create("room_number", igs.INTEGER_T, None)

    igs.start_with_device(sys.argv[2], int(sys.argv[3]))
    
    root_window()
    igs.stop()


