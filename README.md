# Arduino CLI Build Demo

This repository demonstrates using Make to build arduino-cli projects with isolated build tools.

### Mac Setup Prerequisites

You will need to execute the following commands to get a clean install of MaxOSX up and running with the tools necessary to work with this repo. NOTE: This install was done with Sequoia: 15.1.1 and should result in a clean install up and running with the tools necessary to work with this repo.

1. Install Xcode Command Line tools:

    ```  
    xcode-select --install
    ```  

2. Install Rosetta (Only if you are on an M1 Mac):

    ```
    /usr/sbin/softwareupdate --install-rosetta --agree-to-license
    ```


### Getting started
1.  Clone this repository 
2.  Run `make` to get a list of things you can do
    ```
    $ make
    all                  make arduino-cli config-file cores libs sketches
    arduino-cli          Install arduino-cli
    clean                Remove all generated files
    config-file          Create local config file, add BOARD_MANAGER_URLS
    cores                Install the required platform cores
    distclean            Remove all non-versioned files
    libs                 Install required libraries
    sketches             Build all sketches
    nrf52_blink_info     Build nrf52_blink_info
    ```
3.  Run `sudo make arduino-cli` to install arduino-cli (this only has to be done once)
4.  Run `make all` to begin building the sample sketch
    ```
    $ make all
    ```
5.  Edit sample sketch and run `make nrf52_blink_info` to re-build the sample sketch
    ```
    $ vi nrf52_blink_info/nrf52_blink_info.ino
    $ make nrf52_blink_info
    ```


### Uploading code (only tested on Mac)
1.  The following are some examples on how to upload code to a board. The Makefile will attempt to use the first `/dev/cu.usb*` serial port that it finds. A different port can be defined by using the 3rd example below or by setting an environment variable:
    ```
    $ BOARD=sopor:nrf52:whisperpt_3_7 make flash
    $ BOARD=sopor:nrf52:whisperpt_3_7 PORT=/dev/cu.usbmodem101 make flash
    ```
