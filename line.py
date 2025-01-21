import numpy as np
import re
from PIL import Image

img = Image.open('./leet.png').convert('RGB')

def add_margin(pil_img, top, right, bottom, left, color):
    width, height = pil_img.size
    new_width = width + right + left
    new_height = height + top + bottom
    result = Image.new(pil_img.mode, (new_width, new_height), color)
    result.paste(pil_img, (left, top))
    return result
img = add_margin(img,20,10,20,10,(0,0,0)) #Top, Right, Bottom, Left
img = img.resize((60,60))
imageData = np.array(img.getdata())


arrR = imageData[:,0].T
arrG = imageData[:,1].T
arrB = imageData[:,2].T
#print(list(arrR))
#print(list(arrG))
#print(list(arrB))
def toString(arr):
    res = str(arr[0])
    for item in arr[1:]:
        res += ","
        res += str(item)
    return res
stringR = toString(arrR)
stringG = toString(arrR)
stringB = toString(arrR)

resultR = "char imageDataR[] = {{{data}}};".format(data=re.sub(r'\[|\]', '', stringR))
resultG = "char imageDataG[] = {{{data}}};".format(data=re.sub(r'\[|\]', '', stringG))
resultB = "char imageDataB[] = {{{data}}};".format(data=re.sub(r'\[|\]', '', stringB))
result = "\n".join([resultR,resultG,resultB])
with open('./code.txt',"w") as f:
    f.writelines(result)


