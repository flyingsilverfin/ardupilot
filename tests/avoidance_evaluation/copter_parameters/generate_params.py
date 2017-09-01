import os
import re

base_file = "copter_params.parm"

def purge(dir, pattern):
    for f in os.listdir(dir):
        if re.search(pattern, f):
            os.remove(os.path.join(dir, f))

purge('.', 'copter_params_(.)*')

data = open('./' + base_file).read()

for i in range(0, 100):
    outfile = open('copter_params_' + str(i) + '.parm', 'w')
    outfile.write(data)
    outfile.write('\n')
    outfile.write('SYSID_THISMAV\t\t' + str(i))