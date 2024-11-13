
# WORMS Project Codebase

![worms banner](https://github.com/WORMS-OLIGO/worms_ws/assets/67200075/6f45db3f-da1c-4a3d-9e11-7c6b638430c2)

This workspace houses both the **Simulator** and **Hardware Codebases**, all built using **ROS 2 Humble**.

## Cloning the Repository

The first thing you need to do is set up an SSH key on your machine if you have not cloned a workspace before with your Ubuntu VM or dual boot. To do so, enter the following commands:

### 1. Open Terminal

Paste the following text, replacing the email used in the example with your GitHub email address:

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

This will create a new SSH key, using the provided email as a label.

### 2. Generating Public/Private ALGORITHM Key Pair

When you're prompted to "Enter a file in which to save the key", you can press Enter to accept the default file location. 

Please note that if you created SSH keys previously, `ssh-keygen` may ask you to rewrite another key. In this case, we recommend creating a custom-named SSH key. 

To do so, type the default file location and replace `id_ALGORITHM` with your custom key name:

```bash
> Enter a file in which to save the key (/home/YOU/.ssh/ALGORITHM): [Press Enter]
```

At the prompt, do not enter a passphrase, just press the Enter key twice.

```bash
> Enter passphrase (empty for no passphrase): [Press enter]
> Enter same passphrase again: [Press enter]
```

### 3. Testing the SSH Key

To confirm that your key was successfully generated and is correctly set up, use the following command:

```bash
ssh -T git@github.com
```

You should receive a message like this:

```
Hi username! You've successfully authenticated, but GitHub does not provide shell access.
```

If you see this message, you're all set to clone repositories!

---

## WORMS ROS 2 Workspace

The **WORMS ROS 2 workspace** is designed for running both the simulator and hardware codebases of the WORMS project. Below is an overview of the codebase structure:

### Directory Structure:

- **`src/worms_mech`** - Main ROS 2 package for the WORMS project. Contains scripts, launch files, and hardware interfaces.
  - **`launch/`** - Launch files for different ROS nodes.
    - **`camera.launch.py`** - Launch file for camera-related nodes.
    - **`gait.launch.py`** - Launch file for gait/movement-related nodes.
  - **`scripts/`** - Python scripts, such as the hardware interface.
    - **`hardware_interface.py`** - Python script to interact with hardware (e.g., sensors, actuators).
  - **`msg/`** - Custom message types for communication between nodes.
  - **`Testing_Instructions/`** - Instructions for testing the system.
  - **`setup.cfg`** and **`setup.py`** - Configuration files for setting up the package and dependencies.
  - **`package.xml`** - Metadata for the ROS 2 package.
  - **`test/`** - Folder containing tests for the package.

---

## Building the Workspace

Once the workspace is cloned and dependencies are set up, build the entire workspace using the following command:

```bash
colcon build
```

This will compile all the ROS 2 packages in the workspace.

---

## Running the ROS 2 Nodes

Once the workspace is built, you can launch the ROS 2 nodes using the following commands:

### Launching Hardware Interface

To get each worm to it's functional state, you need to run the hardware interface script. This runs the simple_hardware_bootup node, camera, and sets up all the neccesary can setting with the usb to can interfaces. Tjis boot up sequence it powered through a simple bash script. To run it use the following command when in the root directory of the workspace (worms_mech_ws)

```bash
./simple_bootup.sh
```

After this you need to activate the gait manager node on each of the worms. This activates the ROS node responsible for getting commands from the gait manager node that would be send from a chosen commander WORM. To launch that node, use the following:

```bash
ros2 run worms_mech_ws gait_manager
```
After this point you then need to choose a worm to run the gait commander from. This command is found below:

```bash
ros2 run worms_mech_ws gait_action_node
```
Now you are ready to send a command to all connected worms in the network. After the command is entered there will be the option to enter the manuever in which you want the system to do and a list of those can be found below and also in the gait action node file within the repository











### Launching Camera Nodes

To launch the camera-related nodes, use the following command:

```bash
ros2 launch worms_mech camera.launch.py
```

### Launching Gait Nodes

To launch the gait-related nodes, use the following command:

```bash
ros2 launch worms_mech gait.launch.py
```


---

## Installing Dependencies

If you encounter any missing dependencies, use the following command to install them:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Running Tests

To run the package tests, use the following command:

```bash
colcon test
```

---

## Contribution

Feel free to fork the repository and submit pull requests with improvements or bug fixes. Please ensure that your code follows the guidelines provided in the `Testing_Instructions` folder and passes all tests before submitting.

---

## License

The code is licensed under the terms defined in the `LICENSE` file.

---

## Contact

For any questions or issues, please reach out to the project maintainer:

- **Name**: Jacob Rodriguez
- **Email**: jrod@oligo.space

---

## Acknowledgements

This project is built using **ROS 2 Humble** and depends on various other libraries and tools, including those found in the ROS 2 ecosystem.
