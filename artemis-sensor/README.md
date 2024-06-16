# Artemis CPM Firmware

Janet Vorobyeva 2024

This is the firmware for my Cathodic Protection Sensor Mk1
(originally built from `github.com/gemarcano/redboard_template`)

It's built off of Gabe Marcano's fork of the Ambiq SDK
I also am using a forked version of Gabe's asimple library
Build setup forked from `https://github.com/gemarcano/redboard_template`


## Dependencies
 - https://github.com/gemarcano/AmbiqSuiteSDK
 - https://github.com/jvorob/asimple/tree/jv-adc (as of 2024.06.15, may be merged back in later)

In order for the libraries to be found, `pkgconf` must know where they are. The
special meson cross-file property `sys_root` is used for this, and the
`artemis` cross-file already has a shortcut for it-- it just needs a
variable to be overriden. To override a cross-file constant, you only need to
provide a second cross-file with that variable overriden. For example:

Contents of `my_cross`:
```
[constants]
prefix = '/home/gabriel/.local/redboard'
```

# Compiling
```
mkdir build
cd build
# The `artemis` cross-file is assumed to be installed per recommendations from
# the `asimple` repository

meson setup --prefix [prefix-where-sdk-installed] --cross-file artemis --cross-file ../my_cross --buildtype release

# or: (JV: this seems to work fine also, since my_cross specifies prefix)

meson setup --cross-file artemis --cross-file ../my_cross --buildtype release

meson compile (to compile)
```

# Flashing

`meson compile flash` will use sparkfun's SVL bootloader to flash over USB
For my CPS mk1 board, there's no USB UART, so I instead flash using a segger JLINK
and gdb-multiarch. See NOTES.md


# License

See the license file for details. In summary, this project is licensed
Apache-2.0, except for the bits copied from the Ambiq SDK, which is BSD
3-clause licensed.
