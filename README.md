#STM32 Sandbox
This is a sandbox full of stuff/garbage for the STM32.

This repository is currently using:
 * CMSIS v3.01
 * STM32F4xx_StdPeriph_driver v1.0.1

#Setup of development environment (12.04)

## Grab gcc-based toolchain
I'm using the 4.6-2012-q2-update revision from the official GCC ARM launchpad repository:  https://launchpad.net/gcc-arm-embedded

```
sudo apt-get install ia32-libs

cd ~/bin
wget https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q2-update/+download/gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2
tar xvfj gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2
rm gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2.tar.bz2
echo 'export PATH=$PATH:~/bin/gcc-arm-none-eabi-4_6-2012q2/bin:$PATH' >> ~/.bashrc
```

## Build OpenOCD
```
sudo apt-get install libftdi-dev
cd ~/bin
git clone git://github.com/mikeferguson/openocd.git
cd openocd
./bootstrap
./configure --enable-ft2232_libftdi
make
echo 'export PATH=$PATH:~/bin/openocd/src' >> ~/.bashrc
```

## Build DSP_Lib (optional)
In the CMSIS directory, you can build the DSP_Lib by running Make. You may need to change the target processor as it is currently M4lf (Cortex M4, little-endian, with floating point).

# Connecting to a Target
So far I haven't sorted out why openocd hates me, but the following command
works around issues with jimtcl paths:

```
cd ~/bin/openocd/tcl
sudo ../src/openocd  -f interface/flyswatter2.cfg -f target/stm32f4x.cfg
```

Sudo may or may not be neccessary depending on your group configurations.

I tend to use gdb to upload code and interact with the JTAG/STM32. The example makefile has a ".gdbinit" target which exports a .gdbinit file that allows you to run arm-none-eabi-gdb from within the project directory, and exposes a "flash" command to upload firmware, and a "reset" command that works around some quirks in either OpenOCD/Flyswatter/Lack-Of-Moon-Alignment:

```
cd <project>
arm-none-eabi-gdb
> flash
```


