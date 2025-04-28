ensure evalulation works with the alreaady spawned carla car



Later:
    add ppo_train to test the output from ppo, just have some dummy values 
    Some waypoints ar eon sidewalks, cast them to closest lane
    how do we even get the waypoints
    Ensure we follow closest point
    Ensure we can acutally update the closest point
    remove the useless files




pip install carla
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install optuna
pip install gymnasium
pip3 install numpy==1.26.4
pip install matplotlib
pip install opencv-python
pip install stable-baselines3