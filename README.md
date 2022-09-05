# RL-DWA
_Summary: Using stable-baselines3 'PPO' reinforcement learning algorithm to train dynamic window approach_


## :bulb:Installation and Description

### :one: Stable-Baselines3  [[:link:LINK](https://stable-baselines3.readthedocs.io/en/master/)]

For reinforcement learning algorithm, PPO is used in opensource Stable-Baselines3. Click installation link for Stable-Baselines3 above.

Here, the codes are tested in Windows in conda virtual environment dependent on __python 3.7__.

Please keep in mind to match the compatible versions of stable-baselines3, tensorboard, pytorch, python and so on.


### :two: Pygame Environment  [[:link:LINK](https://www.youtube.com/watch?v=Mdg9ElewwA0&t=6s)]

The base idea of creating dynamic window approach **pygame environment** has come from the following link.

In [_scripts/dynamic_window_approach_game.py_](https://github.com/BlackTea12/RL-DWA/blob/main/scripts/dynamic_window_approach_game.py), you can check the modified code.

The main difference is output control of mobile robot changed from __vr, vl__ (right and left wheel angular speed) to __v, w__ (vehicle linear and angular speed).


## :star:Main
In your command prompt (Anaconda Powershell Prompt), execute:

    $ python DWA-learn-main.py
    
There is an option to see just reward log or other train hyperparmeter, loss included logs.
If you wish to see details of the training log, go to [_scripts/DWA_learn_main.py_](https://github.com/BlackTea12/RL-DWA/blob/main/scripts/DWA_learn_main.py) and uncomment line 22 and 28, or just add (if you cannot find):


    c_logger = configure(logdir, ["stdout", "csv", "tensorboard"])
    model.set_logger(c_logger)
    
While training, you can check by executing:

    $ tensorboard --logdir=logs   # simple logs
    $ tensorboard --logdir=${your saved log directory name}   # detail logs
    
## :trolleybus:Algorithm Application in Real World Mobile Robot
Check detailed information in the following link **[HERE](https://github.com/BlackTea12/RL-DWA/tree/main/ros-melodic)**.
