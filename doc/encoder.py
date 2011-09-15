from __future__ import division
from subprocess import *

TERM_WIDTH = 75

def print_bar(value):
    count = int(TERM_WIDTH * value)
    print('-' * count)+'|'+('-'*(TERM_WIDTH-count-1))+"\r",

def get_val():
     while True:
        line = pipe.stdout.readline()
        parts = line.split()

        #print parts
        if parts[1] == '298':
            yield int(''.join([parts[3]]), base=16)

pipe = Popen(['/usr/local/bin/candump', 'can0'], stdout=PIPE)

print ""
for val in get_val():
    print_bar((val &0xff) / 0xff)

