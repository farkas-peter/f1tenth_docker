sudo apt update
sudo apt install software-properties-common apt-transport-https wget -y
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=arm64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install code
code --version

sudo apt install terminator

cd /etc/udev/rules.d/
sudo touch 99-vesc.rules
sudo chmod 646 99-vesc.rules
sudo echo 'KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"' > 99-vesc.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Reboot your system to activate udev rules"
