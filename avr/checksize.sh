#!/bin/bash
codelimit=2048
datalimit=128
TARGET="$1"

set -- $(avr-size -d "$TARGET"| awk '/[0-9]/ {print $1 + $2, $2 + $3, $3}')
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

nm --print-size --size-sort -r --radix=d "$TARGET"
