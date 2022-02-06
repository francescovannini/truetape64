#!/usr/bin/env bash
target="truetape64"
codelimit=2048
datalimit=128

set -- `avr-size -d "$target" | awk '/[0-9]/ {print $1 + $2, $2 + $3, $2}'`
if [ $1 -gt $codelimit ]; then
	echo "Error: code size $1 exceeds limit of $codelimit"
	exit 1
else
	echo "Code: $1 bytes (data=$3)"
fi

if [ $2 -gt $datalimit ]; then
	echo "Error: data size $2 exceeds limit of $datalimit"
	exit 1
else
	echo "Data: $2 bytes"
fi