import os

DATA_DIR = '../data/dataOut'
# get file
out_file = open('hardware-balanced-poses.txt', 'wb')
# Get File Names
file_names = os.listdir(DATA_DIR)

def process_file(text, file_path):
    f = open(file_path, 'r')
    lines = f.readlines()
    for l in lines:
        if l.find("DELETE PREVIOUS LINE") != -1 and len(text) > 0:
            text.pop()
        else:
            text.append(l)

data = []
for f in file_names:
    full_path = os.path.join(DATA_DIR, f)
    if os.path.isfile(os.path.join(full_path)):
        process_file(data, full_path)

out_file.writelines(data)
out_file.close()
