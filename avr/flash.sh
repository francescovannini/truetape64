#!/bin/bash
target="truetape64"
target_hex=$target".hex"
programlimit=2048
datalimit=128

rm -rf cmake-build-release && mkdir cmake-build-release && cd cmake-build-release || exit 1
cmake -DCMAKE_BUILD_TYPE=Release ..
make

set -- `avr-size "$target" | awk '/[0-9]/ {print $1 + $2, $2 + $3, $2}'`
if [ $1 -gt $programlimit ]; then
	echo "Error: program size $1 exceeds limit of $programlimit"
	exit 1
else
	echo "Program: $1 bytes (data=$3)"
fi

if [ $2 -gt $datalimit ]; then
	echo "Error: data size $2 exceeds limit of $datalimit"
	exit 1
else
	echo "Data: $2 bytes"
fi

avr-objcopy -O ihex $target $target_hex
avrdude -c usbtiny -pt2313 -U flash:w:$target_hex -U lfuse:w:0xff:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

