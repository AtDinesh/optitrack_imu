# optitrack_imu

The goal of this program is to use the get data from a xsens device and track this device with an Optitrack IMU.
Reflecting balls have to be placed on the body on which one the IMU is attached and optitrack software should be configured before.

## Getting Started

We will describe here how to launch the ros node.

### Prerequisites

What you need to install beforehand :

* [optitrack](https://wiki.laas.fr/robots/PR2/Mocap?highlight=%28package%29%7C%28require%29%7C%28genomix%29)

## Running

### launch optitrack

In a first terminal: 
```
h2 init
genomixd
```

In another terminal:
```
optitrack-ros
```

In another terminal:
```
elwish
```

In elwish:
```
package require genomix
genomix::connect
genomix1 load <ROBOTPKG_BASE>/lib/genom/ros/plugins/optitrack.so
optitrack::connect {host marey host_port 1510 mcast 239.192.168.30 mcast_port 1511}
```

Make sur motive is broadcasting datas and that the mcast is the good one. 
