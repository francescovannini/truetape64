# TrueTape64 CLI

This tool is part of the TrueTape64 project, aimed at building a simple to build but very reliable interface to dump Commodore 64 tapes into TAP filest suitable to be used with emulators like [Vice](https://vice-emu.sourceforge.io/). Check [here](../) for more infos about the project.

## Installation
    
Be sure to have [pip installed](https://pip.pypa.io/en/stable/installing/), then from the root of this repo, just type
 
    pip3 install .

## Invocation

Simply run the command with no arguments to get help:

    truetape64cli [-d /dev/ttyUSB4] [-y] streetfighter_a.tap
     -d: specify serial device (default is /dev/ttyUSB0) 
     -y: overwrite output file if exists

### Typical output

    $ truetape64cli -d /dev/ttyUSB4 -y streetfighter_a.tap
    PRESS PLAY ON TAPE
    Dumping started
    0 pulses, 0 serial errors, shortest pulse was None
    0 pulses, 0 serial errors, shortest pulse was None
    6 pulses, 0 serial errors, shortest pulse was 26
    13 pulses, 0 serial errors, shortest pulse was 26
    6029 pulses, 0 serial errors, shortest pulse was 20
    16768 pulses, 0 serial errors, shortest pulse was 20

    [...]

    Dumping halted. Dumped 727623 pulses, 0 serial errors, shortest pulse was 2
    Check led status on device

### Practical usage

1) Connect your TrueTape64 adapter to your Commodore Datassette unit
2) Connect your TrueTape64 adapter to one of the USB ports
3) Launch this tool as described above and when prompted...
4) Press PLAY on your Datassette unit
5) Either wait for the tape to reach its end or press STOP when you are done
6) The tool will detect tape activity automatically

If this tool quits before tape reached its end or before you pressed STOP it usually means a hardware error occured during the transfer. This is signaled by a rapidly flashing led on the adapter. In this case press STOP, rewind your tape and start the procedure again. 

The number of serial errors reported by the tool should always be 0. If this number is not zero at the end of the dump, there is a chance that the data has been corrupted or it's missing. If this happens often, replace the FTDI232 adapter with a new one and check the solder joints on your adapter.

## License
[GPLv3](./LICENSE)

