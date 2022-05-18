# 0. ROS 2 publisher

|   | Source code |
|---|----------|
| [`vadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/vadd_publisher) | |
| publisher | [`vadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/vadd_publisher/src/vadd_publisher.cpp) |
| [`doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/doublevadd_publisher) | |
| publisher | [`doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/doublevadd_publisher/src/doublevadd_publisher.cpp) |


This first example presents a trivial vector-add ROS 2 publisher, which adds two vector inputs in a loop, and tries to publish the result at 10 Hz. The ROS 2 package runs in the scalar processors (the CPUs). Step-by-step, the process is documented, walking through the different actions required to run the ROS 2 package in hardware, leveraging KRS capabilities. Afterwards, a slighly modified version of the publisher is presented which has additional computation demands. With these modifications, it becomes clear how the publisher isn't able to meet the publication goal anymore, which motivates the use of hardware acceleration.

The ultimate objective of this example is to generate a simple ROS 2-centric example that creates a CPU baseline to understand the value of hardware acceleration and how KRS facilitates it. Next examples will build upon this one.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to the [install instucctions](install.md)

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `vadd_publisher`

<!-- KRS aims to provide a ROS 2-centric experience (see [here](/features/ros2centric/) for more) and instead of using external tools to build ROS 2 workspaces, extensions to ROS 2 build system (`ament`) and ROS build tools (`colcon`) are implemented which facilitate the process. -->
### Prepare the environment and fetch the example

We start by preparing the environment and fetching the source code of the example into our KRS workspace:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# fetch the source code of examples
$ git clone https://github.com/ros-acceleration/acceleration_examples src/acceleration_examples -b 0.2.0

# build the workspace
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash
```

### Inspecting the ROS 2 publisher

The publisher is a CPU-based average one. The source code of the publisher has been split between the `vadd` function ([vadd.cpp](https://github.com/ros-acceleration/acceleration_examples/blob/main/vadd_publisher/src/vadd.cpp)) and the rest ([vadd_publisher.cpp](https://github.com/ros-acceleration/acceleration_examples/blob/main/vadd_publisher/src/vadd_publisher.cpp)) for simplicity.
The `vadd` (vector-add) function is as follows:

```cpp
/*
        ____  ____
       /   /\/   /
      /___/  \  /   Copyright (c) 2021, Xilinx®.
      \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
       \   \
       /   /
      /___/   /\
      \   \  /  \
       \___\/\___\

Inspired by the Vector-Add example.
See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis

*/

#define DATA_SIZE 4096
// TRIPCOUNT identifier
const int c_size = DATA_SIZE;

extern "C" {
    void vadd(
            const unsigned int *in1,  // Read-Only Vector 1
            const unsigned int *in2,  // Read-Only Vector 2
            unsigned int *out,        // Output Result
            int size                  // Size in integer
            )
    {
        for (int i = 0; i < size; ++i) {
        #pragma HLS loop_tripcount min = c_size max = c_size
            out[i] = in1[i] + in2[i];
        }
    }
}

```

The [`loop_tripcount`](https://www.xilinx.com/html_docs/xilinx2021_1/vitis_doc/hls_pragmas.html#sty1504034367099) is for analysis only and the pragma doesn't impact the function logic in any way. Instead, it allows HLS to identify how many iterations are expected in the loop to make time estimations. This will come handy if we want to run synthesis tests to estimate the timing it'll take for this function to run on a dedicated circuit in the FPGA.

### Building, creating the raw image and running in hardware

Let's build the example, create a raw SD card image and run it in the ultra96v2 board. First, let's select the firmware for the target hardware, ultra96v2:

```bash
$ colcon acceleration select ultra96v2
```

To verify that we indeed that the right firmware selected, look for the one marked with a "*" at the end of its name:
```bash
$ colcon acceleration list
ultra96v2*
```

Let's now build the package targeting the ultra96v2:

```bash
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select ament_vitis vadd_publisher
```

Let's now create a raw disk image for the SD card with PetaLinux's rootfs, a vanilla Linux 5.4.0 kernel and the ROS 2 overlay workspace we've just created for the ultra96v2. First we run a sudo command because the sd image creation needs root privileges so doing this the creation process won't ask for the password in the middle of the process

```bash
$ sudo ls -all
$ colcon acceleration linux vanilla --install-dir install-ultra96v2
```

We're now ready to run it on hardware. For that, we need to flash the `~/krs_ws/acceleration/firmware/select/sd_card.img` file into the SD card. One quick way to do it is as follows:

```bash
# first, find out where your SD card has been mapped, in my case, /dev/rdisk2
$ sudo diskutil umount /dev/rdisk2s1  # umount mounted partition
$ pv <your-path-to>/krs_ws/acceleration/firmware/select/sd_card.img | sudo dd of=/dev/rdisk2 bs=4M  # dd the image
```

There are other methods (for example [Balena Etcher](https://www.balena.io/etcher/)). **Make sure to flash `~/krs_ws/acceleration/firmware/select/sd_card.img` we just generated, and not some other image**.


Once flashed, connect the board to the computer via its USB/UART/JTAG FTDI adapter and power it on. Then, launch your favority serial console (e.g. `sudo putty /dev/ttyUSB1 -serial -sercfg 115200,8,n,1,N`). You should get the prompt (no need a password, it's a root prompt):
(wait about 50 seconds because the system tries to initialize the bluetooth device)
