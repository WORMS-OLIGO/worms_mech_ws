# WORMS Project Codebase
![worms bannet](https://github.com/WORMS-OLIGO/worms_ws/assets/67200075/6f45db3f-da1c-4a3d-9e11-7c6b638430c2)
Workspace to House Simulator and Hardware Codebases. All Built in ROS 2 Humble

### Cloning the Repository

The first thing you need to do is setup an ssh key on your machine if you have not cloned a workspace before with your ubuntu vm/dual boot. To do so enter the following commands:

1) Open Terminal.

Paste the text below, replacing the email used in the example with your GitHub email address.
```
ssh-keygen -t ed25519 -C "your_email@example.com"
```

This creates a new SSH key, using the provided email as a label.

2) Generating public/private ALGORITHM key pair.
When you're prompted to "Enter a file in which to save the key", you can press Enter to accept the default file location. Please note that if you created SSH keys previously, ssh-keygen may ask you to rewrite another key, in which case we recommend creating a custom-named SSH key. To do so, type the default file location and replace id_ALGORITHM with your custom key name.

```
> Enter a file in which to save the key (/home/YOU/.ssh/ALGORITHM):[Press enter]
```
At the prompt, do not enter a passphrase and just press the enter key, repeat twice.

```
> Enter passphrase (empty for no passphrase): [Type a passphrase]
> Enter same passphrase again: [Type passphrase again]
```









Run these commands to install Gazebo:
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
