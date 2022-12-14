#script to syncronize data from 2 signals (tiago joint data and imu matrix data) at different frequencies

import csv 
import numpy as np

test=3
timestamps=[]
imu_timestamps=[]
POE=[]
AOE=[]
HR=[]
FE=[]
PS=[]

if test==1:
  file='./validation/test1/synchro_tiago_imu_test1.csv'
  tiago_data="./validation/test1/TIAGo_LogFile_2022_12_13_12_11_29.csv"
  rot_data="./validation/test1/ROTdata_2022-12-13-12-10-34.csv"

if test==2:
  file='./validation/test2/synchro_tiago_imu_test2.csv'
  tiago_data="./validation/test2/TIAGo_LogFile_2022_12_13_12_26_12.csv"
  rot_data="./validation/test2/ROTdata_2022-12-13-12-25-46.csv"

if test==3: 
  file='./validation/test3/synchro_tiago_imu_test3.csv'
  tiago_data="./validation/test3/TIAGo_LogFile_2022_12_13_12_37_58.csv"
  rot_data="./validation/test3/ROTdata_2022-12-13-12-37-37.csv"


name_file=file
file=open(name_file,'w',encoding='UTF8')
writer_file=csv.writer(file, delimiter=',', lineterminator='\n')
header=['timestamp','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz','POE','UAE','HR','FE','PS']
writer_file.writerow(header)


with open(tiago_data, 'r') as tiago_file:
  csvreader = csv.reader(tiago_file)  
  for row in csvreader:

    if row[0]!='timestamp':
        timestamps=np.append(timestamps, float(row[0])/1000000000) #array that saves all the timestamps
        POE=np.append(POE, float(row[1]))
        AOE=np.append(AOE, float(row[2]))
        HR=np.append(HR, float(row[3]))
        FE=np.append(FE, float(row[4]))
        PS=np.append(PS, float(row[5]))
i=0
print("timestamp length", timestamps.shape)

with open(rot_data, 'r') as imu_file:
  csvreader = csv.reader(imu_file) 
  for timestamp in timestamps:
    for row in csvreader:
        if row[0]!='time':
            if timestamp<=float(row[0]):
                data=[timestamp,row[1],row[2],row[3],row[4],row[5],row[6],row[7],row[8],row[9],row[10],row[11],row[12],row[13],row[14],row[15],row[16],row[17],row[18],row[19],row[20],row[21],row[22],row[23],row[24],row[25],row[26],row[27],POE[i],AOE[i],HR[i],FE[i],PS[i]]
                writer_file.writerow(data)
                break
  
    i=i+1



        
