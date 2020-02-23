# Crazyflie Lighthouse V2 tracking prototype

This repos is an out-of-tree deck driver for the Crazyflie lighthouse deck.

It implements quick-and-dirty proof of concept positioning for lighthouse V2
basestations using the new [lighthouse deck firmware](https://github.com/bitcraze/lighthouse-fpga).

**Warning**: This implementation is just a proof-of-concept, it is noisy and the
measurement ii full of outliers. A proper implementation will be done in the
official [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware).

## System setup

The tracking is done with two SteamVR basestation V2 in modes 1 and 2.
If you want to position the Crazyflie, you can get the basestation geometry from
SteamVR using the script ```python tools/get_bs_position.py``` and copy-paste the
result in ```src/lh2.c``. This is a similar setup as for the Crazyflie in
Lighthouse V1.

## Compilation

The repos needs to be clonned with ```--recursive```. If not, type ```git submodule update --init --recursive``` to get all submodules.

Then, you can build and flash it with ```make && make cload```.

## Test

You can see the elevation and azimuth measurement in the log variables
lh2.elevation* and lh2.azimuth*.
If you have copied the correct basestation geometry earlier, you can see the
lighthouse measured position in lh2.[x,y,z].
The position measurement is sent to the kalman filter so you can also fly
autonomously.

## Limitation

This is a very naive implementation of LH2 pulse tracking and positioning so,
as noted earlier, the resulting positioning is noisy and not very stable.
Furthermore this demo only works with basestation in mode 1 and 2 and the
orientation of the Crazyflie (yaw) is not measured so you should start facing X.

Loss of signal from one basestation is not handled at all so the Crazyflie should
see both basestation at all time.
