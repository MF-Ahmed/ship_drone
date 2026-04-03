# Init antigen.
source "$HOME/.antigen.zsh"

# Configure antigen.
antigen use oh-my-zsh

# Plugins
antigen bundle git
antigen bundle command-not-found
antigen bundle zsh-users/zsh-completions
antigen bundle zsh-users/zsh-syntax-highlighting
antigen bundle srijanshetty/zsh-pip-completion
antigen bundle MichaelAquilina/zsh-auto-notify
antigen bundle unixorn/autoupdate-antigen.zshplugin
antigen bundle iboyperson/pipenv-zsh
# antigen bundle trystan2k/zsh-tab-title
antigen bundle zpm-zsh/undollar
#  antigen bundle mafredri/zsh-async

# Set theme
antigen theme subnixr/minimal

#unset VIRTUAL_ENV
#unset VIRTUAL_ENV_PROMPT

# Apply antigen settings.
antigen apply

# export PS1="%F{green}%n@%m %F{blue}%~%f %F{red}-> %f "

# export PS1="🔥 >"

# Use Python 3 by default.
alias python=python3
alias pip=pip3


export GZ_VERSION=harmonic
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=111

# For crazyfly simulation 
#export GZ_SIM_RESOURCE_PATH="/home/user/data/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo/"   

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gz/models 
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/.gz/models

#export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}/home/user/data/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo/"

#export LIBGL_ALWAYS_SOFTWARE=1

# --- Gazebo / Qt on NVIDIA + X11 (safe defaults) ---
if [[ "$XDG_SESSION_TYPE" == "x11" ]]; then
  unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT MESA_LOADER_DRIVER_OVERRIDE \
        QT_QUICK_BACKEND QT_OPENGL QT_QPA_PLATFORMTHEME GZ_RENDER_ENGINE LD_PRELOAD
  export QT_QPA_PLATFORM=xcb
  export QT_XCB_GL_INTEGRATION=glx
  export QT_OPENGL=desktop
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __NV_PRIME_RENDER_OFFLOAD=1
  export QSG_RENDER_LOOP=basic
fi




#alias r2='source /opt/ros/jazzy/setup.zsh && source /home/user/data/ros2_ws/install/setup.zsh && eval "$(register-python-argcomplete ros2)" && eval "$(register-python-argcomplete colcon)"'

alias r2='source /opt/ros/jazzy/setup.zsh && source /home/user/data/drones_ship_ws/install/setup.zsh && eval "$(register-python-argcomplete ros2)" && eval "$(register-python-argcomplete colcon)"'


source /opt/ros/jazzy/setup.zsh
#source /home/user/data/ros2_ws/install/setup.zsh
#source /home/user/data/ros2_orb_slam3_ws/install/setup.zsh
#source /home/user/data/ls2n_ws/install/setup.zsh
source /home/user/data/drones_ship_ws/install/setup.zsh

eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

export PIP_BREAK_SYSTEM_PACKAGES=1


#export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/user/data/ros2_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
#export GZ_SIM_RESOURCE_PATH=$HOME/uzer/data/ros2_ws/src/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}



if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi


export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/jazzy/lib




# Use the fzf by default
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh





