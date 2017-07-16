# 2017 Robot Code
**Robot Code** | [Dashboard](https://github.com/frc1418/2017-dashboard) | [Image Processing](https://github.com/frc1418/2017-vision)

[![Build Status](https://travis-ci.com/frc1418/2017-robot.svg?token=xpnQFTGBHababzyAzqKV&branch=master)](https://travis-ci.com/frc1418/2017-robot)

This code will be used to control FRC Team 1418's robot during the 2017 challenge, FIRST Steamworks.


## Robot code features

* Full pyfrc integration for testing & robot simulation
* Omni-directional Swerve Drive System
	* Field Centric control 
	* Defensive "wheel locking" stance
* Autonomous mode
	* Score on any side of the tower (consistently at worlds)
	* Strucure allows for both field centric and robot centric drive controls.

## Note

**This swerve drive system is far from ideal. This code is written to field one robot and one robot only. It is not meant to be portable. There tidy few strange things that arise due to our hardware implementation of swervedrive. 3/4 through build season I started to notice this but by that point things just needed to work. I told my team many times that if I were to rewrite swerve drive it would look very different than the system here.**

**TLDR: This system is designed for the irregularities of ONE robot and should not be seen as an over-arching example.**

**- Carter Fendley**

## Deploying onto the robot

The robot code is written in Python, so to run it you must install
[pyfrc](https://github.com/robotpy/pyfrc) onto the robot.

With the pyfrc library installed, you can deploy the code onto the robot
by running robot.py with the following argument:

	python3 robot.py deploy

This will run the unit tests and upload the code to the robot of your
choice.

## Testing/Simulation

The robot code has full integration with pyfrc. Make sure you have pyfrc
installed, and then you can use the various simulation/testing options
of the code by running robot.py directly.

    python3 robot.py sim

## File Structure

    robot/
    	The robot code lives here.
        automations/
            Several automatic scripts for performing common functions like shooting a ball.
        autonomous/
            Autonomous modes.
        common/
            New robotpy components
        components/
            Management of complicated robot systems
	tests/
		py.test-based unit tests that test the code and can be run via pyfrc
    electrical_test/
    	Barebones code ran to make sure all of the electronics are working

## Authors

* [Carter Fendley](https://github.com/CarterFendley)
* [Erik Boesen](https://github.com/ErikBoesen)
* [Dustin Spicuzza](https://github.com/virtuald), mentor
