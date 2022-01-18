# Geometric Model for Serial-Chain Robot Inverse Kinematics in the Case of Two Translational DoF with Spatial Rotation and Task Redundancy


Authors: Moritz Schappler, Tobias Blum, and Tim-David Job

This code is supplemental material to the following submission to the [ARK Conference 2022](https://ark2022.com/):

```bib
@InProceedings{SchapplerBluJob2022,
  author       = {Schappler, Moritz and Blum, Tobias and Job, Tim-David},
  booktitle    = {International Symposium on Advances in Robot Kinematics},
  title        = {Geometric Model for Serial-Chain Robot Inverse Kinematics in the Case of Two Translational DoF with Spatial Rotation and Task Redundancy},
  organization = {Springer},
  year         = {2022},
}
```

## Contents and Useage

This repository contains a Matlab script and Latex code to reproduce results and statements of the paper. Beforehand, the steps from the [prerequesites section](#prerequisites) have to be performed.

### Reproduce Results

The results of the paper can be reproduced by a Matlab script and function templates.

* `calculations/task_redundancy_example.m` of this repo: This creates **Fig. 2** and **Fig. 3** of the paper and the results presented in **Sect. 4**.
* The IK algorithm is implemented in the robotics toolbox (see link below)
  * IK algorithm (**equ. 10-12**): `kinematics/robot_invkin_eulangresidual.m.template`  
    (can be accessed as adapted template for the robot of the paper example by using the debug mode and entering `SerRob/invkin2` and following the function calls; the non-mex function has to be used by `RS.fill_fcn_handles(false)` in the script)
  * Hyperbolic limit criterion (**equ. 13-14**): `kinematics/invkin_optimcrit_limits2.m`
  * Squared joint limit criterion (**equ. 17**): `kinematics/invkin_optimcrit_limits1.m`

## Prerequisites <a name="prerequisites">

For the Matlab scripts to work, the following steps have to be performed:

1. Install Matlab (tested with R2021b)
2. Set up the Matlab mex compiler, if not already done
3. Download Matlab dependencies:
    * [Toolbox for geometry functions](https://github.com/SchapplM/robotics-dep-geometry) (tested with rev. c4af53d; 2021-10-26)
    * [Toolbox for trajectory functions](https://github.com/SchapplM/robotics-dep-traj) (tested with rev. b13e68d; 2021-08-16)
    * [Collection of external dependencies](https://github.com/SchapplM/robotics-dep-ext) (tested with rev. 14712ae; 2021-12-06)
    * [Toolbox for various Matlab functions](https://github.com/SchapplM/matlab_toolbox) (tested with rev. da5759e; 2022-01-04)
    * [Robotics toolbox](https://github.com/SchapplM/robotics-toolbox) (tested with rev. 309b96d; 2022-01-17)
    * [Serial robot model database](https://github.com/SchapplM/robsynth-serroblib) (tested with rev. 67c556b9b; 2022-01-17)
    * Update the repositories to the given versions. If the master branch is on an older version, change of branch is required. You should first try the more recent version before downgrading to the given versions. It should still work.
    * Short command to download all repositories (run in Git Bash in Windows or in Linux terminal):
      ```Bash
      git clone git@github.com:SchapplM/robotics-dep-geometry.git -b dev
      git clone git@github.com:SchapplM/robotics-dep-traj.git -b dev
      git clone git@github.com:SchapplM/robotics-dep-ext.git -b dev
      git clone git@github.com:SchapplM/matlab_toolbox.git -b dev
      git clone git@github.com:SchapplM/robotics-toolbox.git -b dev
      git clone git@github.com:SchapplM/robsynth-serroblib.git -b dev
      ```
4. Run the path initialization scripts (`..._path_init.m`) of all downloaded software repos in Matlab. Put the run commands in the startup.m file of Matlab if used frequently.

