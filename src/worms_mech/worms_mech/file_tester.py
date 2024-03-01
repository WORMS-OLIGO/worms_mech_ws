import numpy as np
import pandas as pd
import subprocess
import platform
import os

def get_mac_address():
    
    
    mac_address = subprocess.check_output(f"cat /sys/class/net/wlan0/address", shell=True).decode().strip()

    if mac_address:
        return mac_address
        
    print("Error getting MAC address: No suitable interface found")
    return None

def find_robot_name(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        return None

def find_species(head, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    print(df['Head'])
    match = df.loc[df['Head'] == head, ['Specialization', 'Motor1_Direction', 'Motor2_Direction', 'Motor3_Direction']]
    if not match.empty:
        return match.iloc[0]
    else:
        return None


# Construct the path to the CSV file for worm info
spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

# Construct the path to the CSV file that holds specialization data
specialization_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/specialization_table.csv')

# Define the path to the file that contains head information from camera node
head_connection_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/head.txt')

# Open the file and read its contents
with open(head_connection_path, 'r') as file:
    head = file.read()

print("HEAD CONNECTOR: " + head)

configuration_info = find_species(head, specialization_path)

print(configuration_info)

