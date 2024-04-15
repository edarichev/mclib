#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 13 12:43:19 2024

@author: de
"""

def makeOneLetter(img, colorLetter, code):
    pixels = list(img.getdata())
    i = 0
    for r in range(0, img.height):
        x = 0
        for c in range(0, img.width):
            pxc = pixels[i][0];
            pxc = pxc * 256 + pixels[i][1];
            pxc = pxc * 256 + pixels[i][2];
            pxc = pxc * 256 + pixels[i][3]
            #print("0x{:08X},".format(pxc))
            color = 0
            if pxc == colorLetter:
                color = 1
            else:
                color = 0
            x = x * 2 + color
            i = i + 1
        print("0x{:02X},".format(x), end='')
    print(' // ', code)
    pass


from PIL import Image
fileName = 'CP866.png'
img = Image.open(fileName, mode='r')

width = 8
height = 8
L = 17
top = 167
code = 0
delta = 11
for i in range(0, 16):
    left = L
    for j in range(0, 16):
        right = left + width
        bottom = top + height
        crImage = img.crop((left, top, right, bottom))
        crImage = crImage.rotate(-90)
        makeOneLetter(crImage, 0xFFFFFFFF, code)
        code = code + 1
        left = left + delta
    top = top - delta

