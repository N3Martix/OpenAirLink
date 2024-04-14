# AirLink: Reproducible Wireless Channel Emulation using Software Defined Radios

An open-source channel emulator for reproducible testing of wireless mobility scenarios.
The emulator implements a FIR filter on Software Defined Radios, the NI USRP (formerly by Ettus). 
AirLink was developed by the [Chair of Communication Networks](https://www.ce.cit.tum.de/lkn/startseite/) at the Technical University of Munich. 

## Requirements
- UHD 4.4

To rebuild FPGA image:
- [Vivado 2021.1](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive.html)
- [AR76780 Patch](https://support.xilinx.com/s/article/76780?language=en_US)

## Installation
Clone this repository:
```
git clone https://github.com/N3Martix/OpenAirLink.git
```
Use following steps to install OpenAirLink:
```
cd ~/OpenAirLink/rfnoc-openairlink
mkdir build && cd build
cmake -DUHD_FPGA_DIR=<path-to-uhd>/uhd/fpga/ ../
make
sudo make install
sudo ldconfig
```
Load FPGA image to your USRP:
```
cd ~/OpenAirLink/fpga-openairlink
uhd_image_loader --args="type=x300" --fpga-path="usrp_x310_fpga_HG.bit"
```
To check installation, run:
```
LD_PRELOAD=usr/local/lib/librfnoc-openairlink.so uhd_usrp_probe
```
If OpenAirLink is correctly installed, the output should look like:
```
|     _____________________________________________________
   |    /
   |   |       RFNoC blocks on this device:
   ...
   |   |   * 0/FIR#0
   |   |   * 0/FIR#1
   |   |   * 0/Shiftright#0
   |   |   * 0/Shiftright#1
   ...
```

## Usage
Lanuch OpenAirLink by:
```
cd ~/OpenAirLink/rfnoc-openairlink/build
LD_PRELOAD=usr/local/lib/librfnoc-openairlink.so ./apps/oal_singel
```
OpenAirLink also supports two channels running independently and simultaneously. To do so, replace 'oal_single' with 'oal_dual'.

The OpenAirLink's channel configuration has two models:

- **Manually**: By default, OpenAirLink periodically scans the configuration file in the `channel_control/` folder to update the channel. The frequency of updates can be adjusted using the '--udt' argument.
- **Script**: The configuration is sent to the USRP if the emulator's running time exceeds its time index. To run the script mode, use the argument `--script`.

## Currently Supported Hardware
1. [NI USRP X310](https://www.ettus.com/all-products/x310-kit/])

## Citing AirLink 

## Contributors 
1. Xianglong Wang
2. Yash Deshpande
