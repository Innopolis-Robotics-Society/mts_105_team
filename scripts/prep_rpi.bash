
# Get keys for docker
sudo apt-get update

sudo apt install -y x11vnc xvfb openbox xterm x11-xserver-utils
mkdir -p ~/.vnc
sudo x11vnc -storepasswd /etc/x11vnc.pass
sudo chmod 600 /etc/x11vnc.pass
Xvfb :1 -screen 0 1920x1080x24 -ac &

sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Get docker
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Get repo
git clone --branch dev https://github.com/Innopolis-Robotics-Society/mts_105_team.git

DISPLAY=:1 openbox-session &
x11vnc -create -usepw -forever -shared -env X11VNC_CREATE_GEOM=1920x1080x24 -display :1 rfbauth /etc/x11vnc.pass

#------------------------

export DISPLAY=:1
xhost +SI:localuser:$(whoami)

# Go to repo
cd mts_105_team/

git submodule init
git submodule update

# Get and start container
sudo docker compose up --build terminal-rpi