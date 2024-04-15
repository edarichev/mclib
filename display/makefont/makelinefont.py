#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 15 09:13:09 2024

@author: de

Дан моноширинный шрифт, символы которого нарисованы в рисунке в одну горизонтальную линию (строку).
Белый - фон. Чёрный - основной цвет. Итого картинка png, монохромная.
Картинка режется на символы, затем построчно - каждый пиксел - это 1 бит,
биты склеиваются подряд блоками по charWidth*charHeight до конца в один
массив типа uint8_t (байты), последние биты дополняются (выравниваются) до 8
Т.о., массив символов сжимается.

Как можно оптимизировать?
Когда-то мы делали так для Java 2 Micro Edition, Если помните.

Пусть у нас есть строка с программе на C++, в тексте используются только определённые
символы, не все 256 и т.п.
Перед выводом на экран эта строка проходится посимвольно,
символ сопоставляется с индексом символа в шрифте.
Например, мы используем только строку "Hello, world!".
Тогда из массива шрифта мы удаляем все символы кроме 
['!', ',', 'H', 'd', 'e', 'l', 'o', 'r', 'w']
Которые расположим в порядке возрастания кода. Пронумеруем это для наглядности:
[ 0,   1,   2,   3,   4,   5,   6,   7,   8 ]
Пусть надо вывести букву 'e'. Индекс буквы мы находим бинарным поиском.
Буква 'e' стоит на 4-м от 0 месте в массиве шрифта, берём биты из массива шрифта,
и выводим соответствующий битмап.
"""

from PIL import Image

# Создать (линейный) список битов шрифта размером (N * charWidth * charHeight) элементов
# Из этого списка можно вынуть только нужные символы
def makeFontBitList(img, charWidth, charHeight, colorLetter = 0x00000000, xLeft = 0, yTop = 0):
    bitlist = []
    x = xLeft
    while x < img.width:
        left = x
        top = yTop
        right = x + charWidth;
        bottom = top + charHeight
        crImage = img.crop((left, top, right, bottom))
        pixels = list(crImage.getdata())
        # построчно - каждый пиксел - это 1 бит
        i = 0
        for r in range(0, charHeight):
            for c in range(0, charWidth):
                pxc = pixels[i];
                color = 0
                if pxc == colorLetter:
                    color = 0
                else:
                    color = 1
                bitlist.append(color)
                i = i + 1
        pass # for r
        x = x + charWidth
    pass # while
    
    return bitlist
# end of makeFontArray

# генерируем байтовый массив C/C++
# В bitlist находятся только необходимые символы
def makeFontArray(bitlist):
    
    # дополнить до 8 бит
    align = len(bitlist) % 8;
    for i in range(0, align):
        bitlist.append(0);
    cStr = "";
    b = 0
    i = 0
    k = 1
    bitnum = 0
    while i < len(bitlist):
        b = b * 2 + bitlist[i];
        print(bitlist[i], end='')
        if i % 100 == 0:
            print('')
        if bitnum == 7:
            cStr += str(b) + ", "
            if k % 10 == 0:
                cStr += '\n'
            b = 0
            bitnum = -1
            k = k + 1
        bitnum += 1
        i = i + 1
        pass
    return cStr

fileName = 'CP866_10x18_line.png'
img = Image.open(fileName, mode='r')

fontBits = makeFontBitList(img, 10, 18)
cppSrc = makeFontArray(fontBits);
content = """// Monospace font 10x18

#ifndef _FONT_MONO_10X18_INCLUDED_
#define _FONT_MONO_10X18_INCLUDED_

static const uint8_t Font10x18[] = {
""" + cppSrc + """};

#endif // _FONT_MONO_10X18_INCLUDED_

"""

outFile = "../fonts/font_mono_10x18.h"
with open(outFile, "w") as f:
    f.write(content)
    pass