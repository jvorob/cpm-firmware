Janet Vorobyeva
2024.04.15
Following notes from here:
https://github.com/gemarcano/AmbiqSuiteSDK

This is a smattering of notes on programming / debugging the Artemis module
via JTAG. First on the Sparkfun "Artemis Redboard ATP", and then
on my own module.


# === QUICKSTART: ====
Window 1: (connect to jtag debugger)
    JLinkGDBServerCLExe -device AMA3B2KK-KBR -speed 4000 -if swd
Window 2: (connect gdb to debugger server from window 1)
    cd ~/school/research/pat/artemis/proj_blinky/build
    gdb-multiarch -x ../gdb_script redboard_template
Window 3: (view SWO Uart output, exposed by gdb)
    netcat localhost 2332


# JTAG Notes
- JLink EDU Mini (segger): downloaded .deb from website

    JLinkGDBServerExe
    JLinkGDBServerCLExe -device AMA3B2KK-KBR -speed 4000 -if swd

    apt install gdb-multiarch

    gdb-multiarch -ex "target extended-remote :2331" redboardElfFile
        monitor SWO EnableTarget 48000000 1000000 1 0
        load    #uploads program
        run

For the current proj, I've preloaded those into a gdb script
    gdb-multiarch -x ../gdb_script redboard_template

## To access USB UART Directly
    microcom --port /dev/ttyUSB0
use CTRL-\ to exit

## To access SWO UART: (through JTAG programmer)
    JLinkSWOViewerCLExe -device AMA3B2KK-KBR -cpufreq 48000000 -swofreq 1000000 -itmport 0
(Or via GDB:)
    netcat localhost 2332

## SWO uart access NOTES:
    https://wiki.segger.com/How_to_use_SWO_with_GDB
    https://wiki.segger.com/J-Link_SWO_Viewer
in gdb:
    monitor SWO EnableTarget 0 0 1 0
in console:
    netcat localhost 2332

command line: (need to press '0' to show port 0 data (printfs))
    JLinkSWOViewerCLExe -device AMA3B2KK-KBR -itmport 0
To be able to keep working after reset, need to specify frequency
    JLinkSWOViewerCLExe -device AMA3B2KK-KBR -cpufreq 48000000 -swofreq 1000000 -itmport 0


# === SDK NOTES: ===

Examples:
    sfAmbiq/

Common Includes:
    am_mcu_apollo.h
    am_bsp.h
    am_util.h

    am_hal_*  (in mcu/apollo3/hal/)
    am_util_*  (in util/)


# Other Docs
Artemis-specific docs: From sparkfun store page, under documents
    https://www.sparkfun.com/products/15442

# AmbiqSDK Install Notes

(note: this is I believe a fork (by gabe) of a fork (by sparkfun) of ambiq's SDK)

to install, can go to AbmiqSuiteSDK/build and do:
    meson install

TODO: I should copy the meson cross-file AmbiqSuiteSDK/artemis
to ~/.local/share/meson/cross/, so I can use it elsewhere?

# LittleFS Install
ATTEMPT 1
cloned littlefs repo
    mkdir build
    # had to comment out CFLAGs something about fcallgraph
    CC=arm-none-eabi-gcc AR=arm-none-eabi-ar BUILDDIR=build make
    cp build/liblfs.a $PREFIX/lib
    cp lfs*.h $PREFIX/include
ATTEMPT 1 didn't work for some reason
ATTEMPT 2: Using Gabe's meson fork of littlefs
https://github.com/gemarcano/littlefs

# UNRELATED: rocketlogger notes
http://yellow-rocketlogger.local/
