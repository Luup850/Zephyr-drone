import pandas as pd
import glob
import os

# Print current working directory
#print(os.getcwd())

# Get all the log files
log_files = glob.glob('..\\drone\\mission\\build\\logs\\*.log')

# Get filename only
log_file_names = [(x.split('\\')[-1])[:-4] for x in log_files]

#for i,l in enumerate(log_file_names):
#    print(l)


# Read in the log file
for i,lg in enumerate(log_files):
    with open(lg, 'rb') as f:
        log = f.readlines()

    # List of bytes to list of strings
    log = [x.decode('utf-8') for x in log]

    header = log[0:5]
    #print(header)

    with open('.\\data\\'+ str(log_file_names[i]) + '-header.log', 'w') as f:
        f.writelines(header)
    #    f.write(header)

    log = log[5:]
    #log = '\n'.join(log)

    with open('.\\data\\'+ str(log_file_names[i]) + '.csv', 'w') as f:
        f.writelines(log)

    # Open the csv as a dataframe
    columns = ['Time', 'X', 'Y', 'Z', 'Pitch', 'Yaw', 'Roll', 'errorHeight', 'errorRoll', 'errorPitch', 'errorYaw']
    #print(len(columns))