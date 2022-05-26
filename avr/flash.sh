#!/bin/bash
target="truetape64"
target_hex=$target".hex"

if [ "$1" == "-b" ]; then
  rm -rf cmake-build-release && mkdir cmake-build-release && cd cmake-build-release || exit 1
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make
  avr-objcopy -O ihex $target $target_hex
else
  cd cmake-build-release || exit 1
fi

avrdude -c usbtiny -pt2313 -U flash:w:$target_hex -U lfuse:w:0xff:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

