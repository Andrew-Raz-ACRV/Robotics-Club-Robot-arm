
import time

import getch
'''
thanks to:
https://pypi.python.org/pypi/getch#downloads

but can only run in the terminal type
python3 keyboardexample.py
'''
x = 0
y = 0
z = 0
p = 0


while True:
    
    ch = getch.getch()
    
    if ch == 'a':
        x = x + 5
    elif ch == 's':
        y = y - 5
    elif ch == 'd':
        x = x - 5
    elif ch == 'w':
        y = y + 5

    print('x = '+str(x)+' y = '+str(y))
        
        

