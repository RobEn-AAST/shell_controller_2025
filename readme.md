# SIMULATION EVALUATION

1. start carla:
     xvfb-run -a /opt/carla/CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -opengl   -graphicsadapter=0   -RenderOffScreen   -benchmark   -nosound   -InitShader -fps=10


2. launch bridge
    roslaunch carla_shell_bridge main.launch
3. launch simualtion
    roslaunch shell_simulation shell_simulation.launch 


# TRAINING
1. cd into ai_src
    cd ~/ros_ws/src/roben_ai/ai_src

2. run opuna_study for getting behyperparameters
    python3 optuna_study.py




REQUIREMENTS TRACKER
pip install flit
CD into GymCarla then execute this: # note i had to do pip3 uninstall alembic
    flit install --symlink

pip3 install ortools

