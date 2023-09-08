import threading
# import concurrent.futures
import serial
import collections
import copy
import time
import joblib
import json
import datetime
import os
import pymap3d as pm
import random
import numpy as np
import pandas as pd 
# from calGps import calGps
# from ubx import UBXManager
from auxiliary import objGetGps
from auxiliary import anchor_gps_2_anc_gps_q
from auxiliary import anc_gps_q_2_anchor_gps

# from readGps import readFixGps
# from readGps import mqtt_readGps
from calUwb import UWBSimulate
from calUwb import UWBHardware
from calUwb import UWBSimulate_enuGPS

from create_teamplate import *

# (lat, lon, heith)
# anchor_gps = [(25.02097, 121.54332, 0), (25.02208, 121.5346, 0), (
#     25.01646, 121.53351, 0), (25.01531, 121.54184, 0)]

z = 1.10
anchor_ENU = [[0, 0, 1.10 ],
                    [22.04, 0, 1.10],
                    [22.04, 5.63, 1.10],
                    [0, 5.63, 1.10]]
positions_list = []
last_time = 0

def calRealPos(offset, pvt_obj, obj_type = 'pyubx2'):
    if obj_type == 'pyubx2':
        year = 2023
        real_pos_obj = pvt_obj  # default
        lat, lon, height = pm.enu2geodetic(
            offset[0], offset[1], offset[2], pvt_obj.lat*10**-7, pvt_obj.lon*10**-7, pvt_obj.height*10**-7)
        lat = round(lat,8)
        lon = round(lon,8)
        height = int(height*10**4)
        print(lat, lon, height)
        msg_NAV_PVT = create_NAV_PVT(lat, lon, height)
        
        positions = np.array([time.time(), lat, lon, height])
        positions_list.append(positions)

        return msg_NAV_PVT
    else:
        real_pos_obj = pvt_obj  # default

        real_pos_obj.lat, real_pos_obj.lon, real_pos_obj.height = pm.enu2geodetic(
            offset[0], offset[1], offset[2], pvt_obj.lat*10**-7, pvt_obj.lon*10**-7, pvt_obj.height*10**-7)

        real_pos_obj.lat = int(real_pos_obj.lat*10**7)
        real_pos_obj.lon = int(real_pos_obj.lon*10**7)
        real_pos_obj.height = int(real_pos_obj.height*10**7)
        real_pos_obj.pDOP = 1
        real_pos_obj.numSV = 15
        
        return real_pos_obj


def sendObj2FC(obj, ser):
    serialized = obj.serialize()
    ser.write(serialized)

dis_to_tag_ls = []
offset_ls = []

def save_data(dis,offset):
    dis_to_tag_ls.append(dis)
    offset_ls.append(offset)

# def write_to_file(dis_filename,offset_filename):
#     dis_to_tag_ls = np.array(dis_to_tag_ls)
#     dis_to_tag_ls = np.transpose(dis_to_tag_ls)
#     df1 = pd.DataFrame({'A2':dis_to_tag_ls[:][0].tolist(),
#                     'A3': dis_to_tag_ls[:][1].tolist(),
#                     'A4': dis_to_tag_ls[:][2].tolist(),
#                     'A5':dis_to_tag_ls[:][3].tolist()})
#     df1.to_csv(dis_filename)                         
#     offset_ls = np.array(offset_ls)
#     offset_ls = np.transpose(offset_ls)
#     df = pd.DataFrame({'x':offset_ls[:][0].tolist(),
#                     'y': offset_ls[:][1].tolist(),
#                     'z': offset_ls[:][2].tolist()})
#     df.to_csv(offset_filename)


if __name__ == '__main__':
    # If you want to run with fix GPS, just initialize 'anchor_gps'
    # No need to run readFixGps
    # ===== Read Port Data =====
    with open("connection_data.json",'r') as f:
        connection_data = json.load(f)[0]
        serCom2FC = connection_data['fc_ttl_com']
        tag_com = connection_data['tag_com']
        anchor_gps = connection_data['anchor_gps'][0]
        uwb_recv_mode = connection_data['uwb_recv_mode']
        isSending2FC = connection_data['isSending2FC']
        save_position_result = connection_data['save_position_result']
        
        
    # ===== Load ubx obj =====
    templatePath = os.path.join(os.path.dirname(__file__), './template')
    # hw_obj = joblib.load(os.path.join(templatePath , 'MON-HW_template.pkl'))
    # hw2_obj = joblib.load(os.path.join(templatePath , 'MON-HW2_template.pkl'))
    # dop_obj = joblib.load(os.path.join(templatePath , 'NAV-DOP_template.pkl'))
    # timegps_obj = joblib.load(os.path.join(templatePath , 'NAV-TIMEGPS_template.pkl'))
    hw_obj = msg_MON_HW
    hw2_obj = msg_MON_HW2    
    dop_obj = msg_NAV_DOP
    timegps_obj = msg_NAV_TIMEGPS


    # ===== determine whether we send the data to FC =====        
    # isSending2FC = True   # set isSending2FC = False to debug without plugging usb_ttl wire
    
    # ===== recording dada initialized =====
    # save_position_result = False
    tagPosData = [] # store the positioning result with the timestamp
    
    # ===== serial initializer =====
    if isSending2FC:
       # ser = serial.Serial(serCom2FC, 230400, timeout=None) #imprtant! 230400
       ser = serial.Serial('/dev/ttyUSB0',230400, timeout=None)
        
    # ===== Anchor pos initialize =====
    # gps: (lat, lon, heith)
    # EE2 experiment =====
    # anchor_gps =[(25.01871570076376, 121.5414674130481, 3.000000000735832), # EE2
    #             (25.018740399874826, 121.5414674130481, 3.000000590359914), 
    #             (25.018740399873735, 121.54148548411611, 3.000000849311539), 
    #             (25.018715700762662, 121.5414854841125, 3.000000259884392)]
    # BL experiment =====
    # anchor_enu = [(0, 0, 0), (-6, 0, 0), (-6, -29, 0), (0, -29, 0)] # Only used when replay (UWBSimulate_enuGPS)
    # anchor_gps = [(25.01941, 121.54243, 1.3), (25.01941, 121.54236, 1.3),
                #   (25.01915, 121.54237, 1.3), (25.01915, 121.54244, 1.3)] 
    # from gps list to ubx obj list   
    anc_gps_q = anchor_gps_2_anc_gps_q(       
        copy.deepcopy(anchor_gps))            # q stores ubx obj
    
    # ===== print initialzed anchor gps data =====
    print('initial data:')
    print('anchor_gps:', anchor_gps)
    print('-------------------------------')
    # print('anc_gps_q:', anc_gps_q)
    
    # ===== mqtt gps pos communication (for multi drone): not yet finished =====
    # th_gps = threading.Thread(
    #     target=mqtt_readGps, args=[anc_gps_q], daemon=True)
    # th_gps.start()
    
    # ===== initialize position containers (global)=====    
    # [(e,n,u)] # tag relitive position to anchor 0
    # tag relative position queue that will be changed by uwbManager
    relPos = collections.deque(maxlen=1)  
    # anc_gps_q = collections.deque(maxlen=1) # Only used when anchors are also moving 
    relPos.append((0, 0, 0))


    
    # ===== set Mode (use recorded uwb data / real time uwb)=====
    # uwb_recv_mode = 'simulate'
    uwb_recv_mode = 'hw'
    if uwb_recv_mode == 'hw':
        uwbManager = UWBHardware(anchor_ENU, relPos, tag_com, anc_gps_q)
    if uwb_recv_mode == 'simulate':         
        uwbManager = UWBSimulate_enuGPS(relPos, os.path.join(os.path.dirname(
            __file__), './uwbData/GPSDe_UWB_dis_robot.json'), anchor_gps)
        
    # ===== initialize uwbManager =====
    uwbManager.start()
    time.sleep(1)

    # ===== Time initialize =====
    start_time = time.time()
    last_time = time.time()
    
    # ===== packet sending state =====
    packet_sending_state = 0 # Alternation of NAV-PVT and NAV-DOP
    # packet_sending_state = 1 # Sending timegps after 5 PVT/DOP
    # packet_sending_state = 2 # Sending HW/HW2 after 5 timegps

    counter = 0

    try:
        pvt_dop_count = 0
        timegps_count = 0
        while True:
            t = time.time()
            # if t - last_time > 1:
            #     print(t - last_time)
            #     last_time = t
            #     duration = t - start_time  # duration: time elapse from start_time to now
                # ===== FSM =====
            if packet_sending_state == 0 :
                if pvt_dop_count < 5:
                    ref_pvt_obj = copy.deepcopy(anc_gps_q[0][0])
                    #offset = relPos[0]
                    #offset = np.array(random.sample(range(1,20),3))
                    offset = uwbManager.offset
                    dis = uwbManager.distanceData
                    save_data(dis, offset)
                    print('offset:', offset)
                    msg_NAV_PVT = calRealPos(offset,ref_pvt_obj)
                    # real_pos_obj = calRealPos(offset, ref_pvt_obj) 
                    # print('Tag position:', objGetGps(real_pos_obj))
                    tagPosData.append([time.time(), dis, offset, msg_NAV_PVT])                    
                    pvt_dop_count = pvt_dop_count + 1
                    # print(msg_NAV_PVT)
                    # print(dop_obj)
                    # print(pvt_dop_count)
                    if isSending2FC:
                        time.sleep(0.18)
                        # sendObj2FC(real_pos_obj, ser)
                        sendObj2FC(msg_NAV_PVT, ser)
                        print('send pvt packet')
                        print(msg_NAV_PVT)
                        time.sleep(0.002)
                        sendObj2FC(dop_obj, ser)
                else:                        
                    pvt_dop_count = 0
                    packet_sending_state = 1
            elif packet_sending_state == 1:
                if timegps_count < 5:
                    # print(timegps_obj)
                    if isSending2FC:
                        time.sleep(0.0471)
                        sendObj2FC(timegps_obj, ser)
                        pass
                    timegps_count = timegps_count + 1
                    packet_sending_state = 0
                if timegps_count == 5:
                    timegps_count = 0
                    packet_sending_state = 2
                    
            elif packet_sending_state == 2:
                # print(hw_obj)
                # print(hw2_obj)
                if isSending2FC:
                    time.sleep(0.008)
                    sendObj2FC(hw_obj,ser)
                    time.sleep(0.13)
                    sendObj2FC(hw2_obj,ser)
                    pass
                packet_sending_state = 0
            # else:
            #     time.sleep(0.01)
    except KeyboardInterrupt:
        print("package")
        uwbManager.stop()
        
    finally:

        print('Save Data!')
        pos = '(7.09,2.418,0.4)'
        dis_filename = './GIPS_uwbData/dis_tag' + pos + '.csv'
        offset_filename = './GIPS_uwbData/offset' + pos +'.csv'
        dis_to_tag_ls = np.array(dis_to_tag_ls)
        dis_to_tag_ls = np.transpose(dis_to_tag_ls)
        df1 = pd.DataFrame({'A2':dis_to_tag_ls[:][0].tolist(),
                        'A3': dis_to_tag_ls[:][1].tolist(),
                        'A4': dis_to_tag_ls[:][2].tolist(),
                        'A5':dis_to_tag_ls[:][3].tolist()})
        df1.to_csv(dis_filename)    

        offset_ls = np.array(offset_ls)
        offset_ls = np.transpose(offset_ls)
        df = pd.DataFrame({'x':offset_ls[:][0].tolist(),
                        'y': offset_ls[:][1].tolist(),
                        'z': offset_ls[:][2].tolist()})
        df.to_csv(offset_filename)

        data_filename = './GIPS_uwbData/uav_lon_lat_alt'+ pos +'.csv'
        positions_ls = np.array(positions_list)
        positions_ls = np.transpose(positions_ls)
        df = pd.DataFrame({ 'time':positions_ls[:][0].tolist(),
                            'lat':positions_ls[:][1].tolist(),
                            'lon': positions_ls[:][2].tolist(),
                            'alt': positions_ls[:][3].tolist()})
        df.to_csv(data_filename)    

        # ===== Save positioning result =====
        if save_position_result == True:            
            filename = f'tagPosData_{datetime.datetime.now().strftime("%Y%m%d-%H%M%S")}.pkl'
            joblib.dump(tagPosData, filename)
            
            



