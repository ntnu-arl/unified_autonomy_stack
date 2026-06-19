https://support.connecttech.com/hc/en-us/articles/26908097848603-Installing-Jetson-Multimedia-API
follow this guide to install the multimedia api on the jetson. This is required for the gst plugins to work.

make install pts_meta_map
with this changes in the makefile
# Installation directories
PREFIX ?= /usr/lib/aarch64-linux-gnu/gstreamer-1.0
LIBDIR := $(PREFIX)
INCDIR := $(PREFIX)/include

sudo apt install v4l-utils
v4l2-ctl -d /dev/video2 -c trigger_mode=1 for all cameras

The Above is on host!!!!!!!!!!!!!!!!

In the dockerfile for ros2_base, add the following lines to install the pts_meta_map package:
# Get and install the custom pts_meta_map
RUN git clone https://github.com/ntnu-arl/pts_meta_map.git /tmp/pts_meta_map && \
    cd /tmp/pts_meta_map && \
    make install

changed ros_gst_bridge to dev/exposure_time + some local uncommited changes on Parrot
remove the printf in source_driver_ros1.hpp and source_driver_ros2.hpp from the HesaiROS driver because is anoying!

ip adress in the jt128.yml to 192.168.1.201


# PTP Syncronization (on Host)
sudo apt install linuxptp

enable service: sudo systemctl enable ptp4l
start service: sudo systemctl start ptp4l

/etc/linuxptp/ptp4l.conf 
last line in ptp4l.conf should be: [enP7p1s0] or [enP8p1s0] on NVidia Jetson Orin NX,

# Maybe optional, but to make sure the clock is syncronized, you can also run the phc2sys service:
ExecStart=/usr/sbin/phc2sys -w -c CLOCK_REALTIME -s enP7p1s0 -S 0.1 -O 0


THS2->THS1 for IMU (only on this module)
