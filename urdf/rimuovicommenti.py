import re
import os

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

print(os.getcwd())
f = open("ciao.urdf", "r")
a = f.read()
b = remove_comments(a)
print(b)
f.close()





