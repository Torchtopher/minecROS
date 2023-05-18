FROM ros:noetic-ros-base
WORKDIR /home/ubuntu
RUN apt update && apt install -q -y --no-install-recommends \
    apt-utils \
    scrot \
    sudo \
    git \
    python3-dev \
    build-essential \
    whois \   
    python3-pip \    
    python3-tk &&\
    rm -rf /var/lib/apt/lists/*
RUN apt update && apt install --no-install-recommends -y \
        build-essential \
        python3-pip \
        python3-rosdistro \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-vcstools &&\
     pip3 install catkin_tools rosdep vcstool &&\
     rm -rf /var/lib/apt/lists/*
RUN pip3 install pyautogui pynput

# add ubuntu user
RUN useradd -ms /bin/bash -G sudo -p `mkpasswd -m sha-512 ubuntu` ubuntu &&\
    chown ubuntu:ubuntu /home/ubuntu &&\
    adduser ubuntu dialout

# Set up various paths in .bashrc here
RUN echo "source /home/ubuntu/minecROS_ws/command_aliases.sh" >> /root/.bashrc &&\
    echo "export PATH=\$PATH:/usr/games/bin" >> /root/.bashrc &&\
    echo "# enable bash completion in interactive shells" >> /etc/bash.bashrc &&\
    echo "if ! shopt -oq posix; then" >> /etc/bash.bashrc &&\
    echo "  if [ -f /usr/share/bash-completion/bash_completion ]; then" >> /etc/bash.bashrc &&\
    echo "    . /usr/share/bash-completion/bash_completion" >> /etc/bash.bashrc &&\
    echo "  elif [ -f /etc/bash_completion ]; then" >> /etc/bash.bashrc &&\
    echo "    . /etc/bash_completion" >> /etc/bash.bashrc &&\
    echo "  fi" >> /etc/bash.bashrc &&\
    echo "fi" >> /etc/bash.bashrc

RUN cp /root/.bashrc /home/ubuntu/.bashrc &&\
    chown ubuntu.ubuntu /home/ubuntu/.bashrc

USER ubuntu
