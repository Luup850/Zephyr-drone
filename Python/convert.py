import pandas as pd
import glob
import os
import numpy as np

def convert_drone_log_to_csv():
    # Print current working directory
    #print(os.getcwd())

    # Get all the log files
    log_files = glob.glob('..\\drone\\mission\\build\\logs\\*.log')

    # Get filename only
    log_file_names = [(x.split('\\')[-1])[:-4] for x in log_files]
    #for i,l in enumerate(log_file_names):
    #    print(l)
    columns = ['Time', 'X', 'Y', 'Z', 'Pitch', 'Yaw', 'Roll', 'errorHeight', 'errorRoll', 'errorPitch', 'errorYaw', 'HLeadOut', 'HIntegralOut', 'HeightError']

    # Read in the log file
    for i,lg in enumerate(log_files):
        data = []
        oldFormat = False
        initial = []
        with open(lg, 'rb') as f:
            log = f.readlines()

        if(log_file_names[i] == 'Wed May  3 14_25_54 2023'):
            # Special case: Remove first line and last line
            log = log[1:-1]
        else:
            # Remove last line
            log = log[:-1]

        # List of bytes to list of strings
        log = [x.decode('utf-8') for x in log]

        header = log[2:4]

        # Check if file is empty
        if(len(log) < 6):
            print('Log file ' + str(log_file_names[i]) + ' is empty')
            # Delete file
            os.remove(lg)
            continue
        
        if((header[0]).split(',')[0] != 'HOVER_VALUE'):
            print('Log file ' + str(log_file_names[i]) + ' uses old format')
            oldFormat = True
        


        if(oldFormat == False):
            log = log[5:]
        else:
            log = log[2:]

        # Check that there are more than 20 lines
        if len(log) < 20:
            print('Log file ' + str(log_file_names[i]) + ' is too short')
            return

        if(oldFormat == False):
            #print(log_file_names[i])
            header_column = header[0].split(',')
            # Remove spaces
            header_column = [x.strip() for x in header_column]
            header_data = header[1].split(',')[:-1]
            header_data = [x.strip() for x in header_data]
            # Create dataframe
            #print(header_column)
            df_header = pd.DataFrame([header_data], columns=header_column)
            df_header.to_csv('.\\data\\'+ str(log_file_names[i]) + '-header.csv', index=False)
        else:

            # Generate empty header
            header_column = ['HOVER_VALUE', 'ctrl_x->ref', 'ctrl_y->ref', 'ctrl_yaw->ref', 'ctrl_h->ref', 'ctrl_h->kp', 'ctrl_h->tau_i', 'ctrl_h->tau_d', 'ctrl_h->alpha', 'ctrl_vel_x->tau_i', 'ctrl_vel_x->tau_d', 'ctrl_vel_x->alpha']
            header_data = ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']
            # Create dataframe
            df_header = pd.DataFrame([header_data], columns=header_column)
            df_header.to_csv('.\\data\\'+ str(log_file_names[i]) + '-header.csv', index=False)


        # split on ,
        for j,l in enumerate(log):
            l = l.split(',')
            if(len(l) != 14):
                # append zeroes until it is 14
                while(len(l) != 14):
                    l.append('0.0')

            data.append(l)
        # Save the dataframe as a csv
        df = pd.DataFrame(data, columns=columns)
        df.to_csv('.\\data\\'+ str(log_file_names[i]) + '-log.csv', index=False)

def convert_aruco_log_to_csv():
    log_files = glob.glob('..\\drone\\mission\\build\\aruco_logs\\*.log')
    
    # Get filename only
    log_file_names = [(x.split('\\')[-1])[:-4] for x in log_files]
    
    # if folder data_aruco does not exist, create it
    if not os.path.exists('.\\data_aruco'):
        os.makedirs('.\\data_aruco')

    # Read in the log file
    for i,lg in enumerate(log_files):
        data = []
        keys = []
        with open(lg, 'rb') as f:
            log = f.readlines()

            # If file has less than 4 lines, skip it
            if(len(log) < 4):
                print('Log file ' + str(log_file_names[i]) + ' is empty')
                continue

            for j,l in enumerate(log):
                # Remove linebreak at the end
                #l = l[:-1]
                if (j == 0):
                    print("First line")
                elif (j == 1):
                    entry = l.split(b',')
                    # Strip last element of '\n'
                    entry = [x.decode('utf-8') for x in entry]
                    entry[-1] = entry[-1][:-1]
                    keys.append(entry)
                else:
                    entry = l.split(b',')
                    # Remove any 'b'
                    entry = [x.decode('utf-8') for x in entry]
                    # Strip last element of '\n'
                    entry[-1] = entry[-1][:-1]
                    # Strip for any spaces
                    entry = [x.strip() for x in entry]
                    data.append(entry)

        # Save the dataframe as a csv
        # Remove any spaces in keys
        keys = [x.strip() for x in keys[0]]
        df = pd.DataFrame(data, columns=keys)
        df.to_csv('.\\data_aruco\\'+ str(log_file_names[i]) + '-aruco_log.csv', index=False)