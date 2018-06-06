# QUICKSTART for setup and run in Ubuntu

### Install dependencies

    sudo apt-get install morse-simulator python3-morse-simulator

    sudo apt-get install redis-server

    pip3 install redis

### Install menelaus

    cd <directory where you wish to store the repository>

    git clone https://github.com/ekrell/menelaus.git
    
    export PYTHONPATH="${PYTHONPATH}:<chosen directory>/menelaus/lib"
    
### Startup robots

Each robot is of a particular robot type. Instances of these robots are specified in a MORSE scene file. 
For example, the file __menelaus/scenes/field_exercise_1.py__ specifies three ground robots of type MARISA and a single quadcopter of type QCOORD.
The three MARISA robots are named Susan, Django, and anton. The QCOORD is named Godot.  
Additional robots can easily be added by following the example scene scripts in __menelaus/scenes__. 

A robot is started by running the python script that corresponds to the robot type, 
such as __menelaus/robots/marisa.py__ or __menelaus/robots/qcoord.py__.
The name of the particular robot is a parameter to the script so that it knows which robot to control. 

I typically have a single terminal window up for each robot, since each outputs information to stdout. 

In the following demonstrationg, two MARISA robots will be started with a list of waypoints to navigate to. 
The QCOORD will also be initiated. 
Another python script represents a ground control station reading each robot's messages and using that information
to coordinate a follow behavior. 

    cd <chosen directory>/menelaus
    
**Launch Morse scene**

    morse run scenes/field_exercise_1.py

**Start a MARISA robot named susan**

    python3 robots/marisa.py -n susan -w inData/round10/susan.waypoints

**Start a MARISA robot named anton**

    python3 robots/marisa.py -n anton -w inData/round10/anton.waypoints
    
**Start a QCOORD robot named godot**
    
    python3 robots/qcoord.py -n godot
    
**Initiate follow behavior**

    python3 scripts/follow_targets.py -q godot -t susan,anton
    
  
    
    
