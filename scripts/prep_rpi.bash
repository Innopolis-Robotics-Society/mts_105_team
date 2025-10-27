
# Get keys for docker
sudo apt-get update

sudo apt install -y x11vnc xvfb xauth openbox xterm x11-xserver-utils
Xvfb :0 -screen 0 1920x1080x60 -ac 
DISPLAY=:0 openbox-session &
mkdir -p ~/.vnc
x11vnc -storepasswd ~/.vnc/passwd
chmod 600 ~/.vnc/passwd
export DISPLAY=:0
xhost +SI:localuser:$(whoami)
# xterm : To debug
x11vnc -create -usepw -forever -shared -env X11VNC_CREATE_GEOM=1920x1080x60


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

x11vnc -create -usepw -forever -shared -env X11VNC_CREATE_GEOM=1920x1080x24

#------------------------

export DISPLAY=:1
xhost +SI:localuser:$(whoami)

# Go to repo
cd mts_105_team/

git submodule init
git submodule update

# Get and start container
sudo docker compose up --build terminal-rpi
