#!/usr/bin/env bash

# Basic GDIP example
rm -rf images
./basic_gdip_example.jl -save
convert -delay 7 -loop 0 images/*.png basic-gdip-example.gif
convert basic-gdip-example.gif -fuzz 5% -layers Optimize basic-gdip-example-small.gif

# RSS'18 - DTRP example
rm -rf images
./rss18_dtrp.py -save
convert -delay 50 -loop 0 images/*.png rss-example.gif
convert rss-example.gif -fuzz 5% -layers Optimize rss-example-small.gif
