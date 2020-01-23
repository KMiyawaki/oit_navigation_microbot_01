#!/bin/bash

INPUT_FILE=${1}
OUTPUT_FILE=${INPUT_FILE%.*}_border.png
convert ${INPUT_FILE} -bordercolor "#000000" -border 5x5 ${OUTPUT_FILE}
