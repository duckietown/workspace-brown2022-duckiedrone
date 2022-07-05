# workspace-duckiedrone-core

Workspace used to work on the duckiedrone DD21 for `brown2022`.


## Modules

We call "modules" the software components that consitute the full
autonomous flight stack.
These modules are:

- **Altitude**: Combines IMU and ToF readings to compute drone's altitude;
- **State Estimation**: Estimate the drone's position by combining data from various sensors;
- **Flight Controller**: Controls the robot's action in order to achieve a desired behavior (e.g., hover in place, move forward, etc.);
- **Optical Flow**: Estimate drone's movement from changes in the camera frame;


## Open all modules' terminals

Press <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>P</kbd> to open the
command palette then select "Terminals: Run" and press <kbd>Enter</kbd>.

You will see a terminal popup from the bottom edge of VSCode, and a 
list of terminal tabs, one for each component of the autonomous flight 
pipeline, will appear on the right side.

NOTE: If you happen to accidentally close one or more of your
terminal tabs, simply re-run the command "Terminals: Run" from the
command palette to recover the missing tabs.

NOTE: If you happen to accidentally mess up one of your terminals, simply
close it and re-run the command "Terminals: Run" from the
command palette to obtain a brand new terminal environment.


## Implement a module

From the `Explorer` (left-hand side in VSCode), navigate to the directory
`src/` inside the `BROWN2022` workspace.
Each module `<M>` is implemented inside `src/` as a ROS package.
Each ROS package in this workspace comprises of a ROS node in 
`src/<M>/src/<M>_node.py` and a ROS launch file in 
`src/<M>/launch/<M>_node.launch`.

The best way to implement a new ROS package is to focus on the 
node first and then implement the integration with the rest of the
ROS network using the options available in the launch file.

For example, for the `altitude` module, you will 
implement your code inside the python file 
`src/altitude/src/altitude_node.py` 
and define the integration with the ROS network inside the launch file
`src/altitude/launch/altitude_node.launch`.


## Run a module

### Default VS Student mode

Each module comes pre-implemented by Duckietown and you can run them
on your drone out-of-the-box. This is nice but it doesn't teach us much, 
so we will go in and implement each module ourselves, one by one.

For each module, you can decide to run the default implementation 
(as provided by Duckietown), or your own (i.e., the one you implement 
in VSCode).

In order to run a module, navigate to the terminal tab that belongs to 
the module you want to run. You can do so by clicking on the module's 
name from the terminal tabs list on the right-hand side of VSCode.

NOTE: These are smart terminal tabs, and while they might look similar 
to each other, they are each configured to work on a different module.

### Run the default implementation

Run the following command in the terminal to run a module's default 
implementation,

```shell
dtm-run
```


### Run your own implementation

Run the following command in the terminal to run your own
implementation of a module,

```shell
dtm-run --student
```


