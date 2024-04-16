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

###############################################################################

def cutSymbols(text, fontBits, charWidth, charHeight):
    # вставляем в порядке возрастания кода символа сам символ и его битмап, если его ещё нет
    minBits = []
    minFontCharCodes = []
    for ch in text:
        chCode = ord(ch)
        x = chCode * charWidth * charHeight
        pos = 0
        while pos < len(minFontCharCodes):
            if minFontCharCodes[pos] == chCode:
                pos = -1 # уже есть, не нужно
                break
            if minFontCharCodes[pos] > chCode:
                break
            pos = pos + 1
        if pos == -1:
            continue
        minFontCharCodes.insert(pos, chCode)
        minBits.insert(pos, fontBits[x:x + charWidth * charHeight])
        print(minFontCharCodes)
    pass # for
    merged_list = []
    for sublist in minBits:
        merged_list.extend(sublist)
    return (minFontCharCodes, merged_list)

###############################################################################

def generateMinFontRemap(minFontData):
    strSrc = ""
    i = 1
    for chCode in minFontData[0]: # char codes
        strSrc += "0x{:02X}".format(chCode) + ", "
        if i % 10 == 0:
            strSrc += "\n"
        i = i + 1
    return strSrc

###############################################################################

fileName = 'CP866_10x18_line.png'
img = Image.open(fileName, mode='r')

charWidth = 10
charHeight = 18

fontBits = makeFontBitList(img, charWidth, charHeight)
cppSrc = makeFontArray(fontBits);
strXDim = str(charWidth) + """x""" + str(charHeight)
strGuard = "_FONT_MONO_" + strXDim + "_INCLUDED_"
content = """// Monospace font """ + strXDim + """

#ifndef """ + strGuard + """
#define """ + strGuard + """

const uint8_t Font""" + strXDim + """[] = {
""" + cppSrc + """};

#endif // """ + strGuard + """

"""

outFile = "../fonts/font_mono_10x18.h"
with open(outFile, "w") as f:
    f.write(content)
    pass

# Только необходимые символы

text = "Hello, world"
minFontData = cutSymbols(text, fontBits, charWidth, charHeight)
cppSrc = makeFontArray(minFontData[1]);

outFile = "../fonts/font_mono_10x18_min.h"
strGuard = "_FONT_MONO_" + strXDim + "_MIN_INCLUDED_"
content = """// Monospace minimized font """ + strXDim + """

#ifndef """ + strGuard + """
#define """ + strGuard + """

const uint8_t Font""" + strXDim + """_min[] = {
""" + cppSrc + """};

"""
cppSrc = generateMinFontRemap(minFontData)

content += """const uint8_t Font""" + strXDim + """_min_remap[] = {
""" + cppSrc + """};

#define Font""" + strXDim + """_min_remap_char_count (sizeof(Font""" + strXDim + """_min_remap) / sizeof(uint8_t))
#endif // """ + strGuard + """

"""

with open(outFile, "w") as f:
    f.write(content)
    pass
