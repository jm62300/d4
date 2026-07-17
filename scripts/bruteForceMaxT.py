import re
import sys
from subprocess import Popen, PIPE

counterPath="./d4_static"
counterOption=["-m", "counting", "-f", "1", "-i"]
FILE_NAME = '/tmp/bruteTest.cnf'

f = sys.argv[1]

maxVar = []
indVar = []

instance = ""


with open(f, 'r') as file:
    for line in file:
        if line[0] != 'c':
            instance += line
        else:
            if re.search("c max", line):
                maxVar += [int(v) for v in line.split(' ')[2:-1]]
            elif re.search("c ind", line):
                indVar += [int(v) for v in line.split(' ')[2:-1]]
            else:
                instance += line

maxVar.sort()

instance += "c p show "
for v in maxVar: instance += str(v) + ' '
for v in indVar: instance += str(v) + ' '
instance+='0\n'
first = True
maxCount = 0
maxVal = []

for i in range(pow(2, len(maxVar))):
    tmp = i
    assignment = ''
    val =[]
    for j in range(len(maxVar)):

        if tmp % 2 == 0:
            assignment += str(maxVar[j]) + " 0\n"
            val += [maxVar[j]]
        else: 
            assignment += str(-maxVar[j]) + " 0\n"        
            val += [-maxVar[j]]
        tmp //= 2


    tmpInstance = instance + assignment
    with open(FILE_NAME, 'w') as f:
        print(instance, file=f)
        print(assignment, file=f)


    process = Popen([counterPath] + counterOption + [FILE_NAME], stdout=PIPE)
    (output, err) = process.communicate()
    exit_code = process.wait()

    for line in str(output).split('\\n'):
        if re.search("^s ", line):            
            count = line.split()[1]

            if first or count > maxCount:
                maxCount = count
                maxVal = val
                first = False                



print('v', end=' ')
for v in maxVal: print(v, end=' ')
print()

print('o', maxCount)