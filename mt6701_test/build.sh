#!/bin/bash
# MT6701 cozumleme birim testi (PC, gcc) + CMSIS ile sozdizimi kontrolu
set -e
cd "$(dirname "$0")"
ROOT=..
OUT=build; mkdir -p $OUT
INC="-I$ROOT/Core/Inc -I$ROOT/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I$ROOT/Drivers/CMSIS/Include"

echo "== Host test =="
gcc -Wall -Wextra -Werror -O2 -DMT6701F_HOST_TEST -I$ROOT/Core/Inc test_host.c $ROOT/Core/Src/mt6701_decode.c -o $OUT/test_host
$OUT/test_host

SRCS="$ROOT/Core/Src/mt6701_fast.c $ROOT/Core/Src/mt6701_decode.c $ROOT/Core/Src/foc_encoder_mt6701.c"
if command -v arm-none-eabi-gcc >/dev/null; then
    echo "== ARM derleme =="
    for f in $SRCS; do
        arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -Wall -Wextra \
            -DSTM32F407xx $INC -c $f -o $OUT/$(basename ${f%.c}).o
    done
    arm-none-eabi-size $OUT/*.o
else
    echo "== Sozdizimi kontrolu (arm-none-eabi-gcc yok) =="
    for f in $SRCS; do
        gcc -fsyntax-only -Wall -Wextra -Wno-int-to-pointer-cast -DSTM32F407xx $INC $f
    done
    echo "OK"
fi
