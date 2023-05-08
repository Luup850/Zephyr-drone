import pandas as pd
import glob
import os
import numpy as np

def convert_log_to_csv():
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

        # List of bytes to list of strings
        log = [x.decode('utf-8') for x in log]

        header = log[2:4]
        if(header[0].split(',')[0] != 'HOVER_VALUE'):
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

convert_log_to_csv()