
# -*- coding: UTF-8 -*-
import serial
import glob
import copy
import json
import threading
import collections
import datetime
import time
import os
import numpy as np
import pymap3d as pm
import collections
from auxiliary import anc_gps_q_2_anchor_gps
from datetime import datetime
from distance2Position import costfun_method
from distance2Position import lsq_method
from distance2Position import my_lsq
from distance2Position import chan_algo
from algorithm import two_stage
from uwbReceive import UWB_dis
distanceQ = collections.deque(maxlen=1)

# class UWBBase():
#     def __init__(self):
#         self.


class UWBHardware():
    def __init__(self, anchor_ENU, offsetQ, comport, anchor_gps_q, baudrate=115200, timeout=0.01,):
        # self.ser = serial.Serial(comport, baudrate=baudrate, timeout=timeout)
        self.is_run = threading.Event()
        self.distanceData = [0, 0, 0, 0]
        self.offset = [0,0,0]
        self.ref_point_ENU = anchor_ENU[0]
        
        self.anchor_gps_q = anchor_gps_q
        self.anchor_gps = copy.deepcopy(
            anc_gps_q_2_anchor_gps(anchor_gps_q))  # gps list
        self.anchorPosition_enu = []
        self.offsetQ = offsetQ
        self.uwbDataList = []
        self.start_time = time.time()
        # self.calibrate_dis = np.array([-.6, -.6, -.6 ,-.6])
        self.calibrate_dis = np.array([-0.6, -0.6, -0.6, -0.6])
        # self.recordDistanceData = []
        self.prev_dis = np.array([.0, .0, .0, .0])

        # for two-stage algorithm
        self.anchor_ENU = np.array(anchor_ENU)
        self.anchor_offset = self.anchor_ENU[0]
        self.A = self.anchor_ENU[1:] - self.anchor_offset
        self.u, self.s, self.vh = np.linalg.svd(self.A, full_matrices=True)

    def gps2enu_list(self, anchorPosition_gps):
        # print(anchorPosition_gps)
        anchorPosition_enu = [(0, 0, 0)]
        refPointGps = anchorPosition_gps[0]
        for i in range(1, len(anchorPosition_gps)):
            # print(anchorPosition_gps[i][0])
            enu = pm.geodetic2enu(anchorPosition_gps[i][0], anchorPosition_gps[i][1],
                                  anchorPosition_gps[i][2], refPointGps[0], refPointGps[1], refPointGps[2])
            anchorPosition_enu.append(enu)
        # print(anchorPosition_enu)
        self.anchorPosition_enu = anchorPosition_enu

    def findPos(self):
        two_stage_result, det, b = two_stage(self.distanceData, self.anchor_ENU, self.u)
        self.offset = two_stage_result[0] - self.ref_point_ENU

    def onUwb(self):

        self.distanceData = [0, 0, 0, 0]
        self.findPos()
        # COM_PORT = '/dev/ttyUSB1'
        # rcv_dis = UWB_dis(COM_PORT)
        # self.distanceData = rcv_dis
        # print('rcv_dis:',self.distanceData)
        # #self.findPos()

        # if(0 not in rcv_dis):
        #     self.distanceData = rcv_dis
        #     print('enter rcv_dis:',self.distanceData)
        #     self.findPos()
            
        '''
        offset = [0,0,0]
        self.offsetQ.append(offset)
        
        # self.recordDistanceData.append()
        '''

    def run(self):
        fi_num = datetime.now().strftime("%H_%M_%S")
        self.start_time = time.time()
        print('ref point:',self.ref_point_ENU)
        
        while self.is_run.is_set():
            # if (self.ser.inWaiting() > 0):
            #     rx_1 = self.ser.readline()

            #     if (len(rx_1) >= 20 and 'mc' in str(rx_1)):
            #         self.onUwb(rx_1)
            # # time.sleep(0.2)
            self.onUwb()
        

    def start(self):
        self.is_run.set()
        threading.Thread(target=self.run, daemon=True).start()

    def stop(self):
        self.is_run.clear()
        # self.ser.close()
        filename = os.path.dirname(
            __file__)+'/uwbData/UWB_dis_' + datetime.now().strftime("%Y%m%d-%H%M%S") + '.json'
        with open(filename, 'a') as fout:
            json.dump(self.uwbDataList, fout)
        print('finish dumping ubx json data.')


class UWBSimulate():
    def __init__(self, offsetQ, filename, anchor_gps):
        self.is_run = threading.Event()
        self.distanceData = [0, 0, 0, 0]
        self.anchor_gps = copy.deepcopy(anchor_gps)
        self.anchorPosition_enu = []
        self.offsetQ = offsetQ
        with open(filename, 'r') as f:  # os.path.dirname(__file__)+'/uwbData/UWB_dis_18_49_17.json'
            self.allUwb = json.load(f)
        # print(self.allUwb)

    def metadata_initialize(self, anchorPosition_gps):
        print(anchorPosition_gps)
        anchorPosition_enu = [(0, 0, 0)]
        refPointGps = anchorPosition_gps[0]
        for i in range(1, len(anchorPosition_gps)):
            # print(anchorPosition_gps[i][0])
            enu = pm.geodetic2enu(anchorPosition_gps[i][0], anchorPosition_gps[i][1],
                                  anchorPosition_gps[i][2], refPointGps[0], refPointGps[1], refPointGps[2])
            anchorPosition_enu.append(enu)
        print('metadata_initialize:anchorPosition_enu:\n', anchorPosition_enu)
        self.anchorPosition_enu = anchorPosition_enu

    def uwbReplay(self):
        # print('uwbReplay, self',self.anchor_gps)
        # print(self.anchor_gps)
        self.metadata_initialize(self.anchor_gps)
        start_time = time.time()
        for uwb in self.allUwb:
            timestamp = float(uwb['time'])
            while time.time()-start_time < timestamp:
                time.sleep(0.01)
            try:
                self.onUwb(uwb)
            except Exception as e:
                print(e)
                self._onUwb(uwb)
        self.stop()

    def _onUwb(self, uwb):
        print(uwb)

    def onUwb(self, uwb):
        # print(uwb)
        self.distanceData = [float(i)/1000 for i in uwb['dis'][:4]]
        offset = costfun_method(self.distanceData, self.anchorPosition_enu)
        # print('offset:',offset)
        self.offsetQ.append(offset)

    def start(self):
        self.is_run.set()
        threading.Thread(target=self.uwbReplay, daemon=True).start()

    def stop(self):
        self.is_run.clear()


class UWBSimulate_enuGPS(UWBSimulate):
    # In UWBSimulate_enuGPS we use key-in enu positions,
    # not real enu transformed from GPS.
    def __init__(self, offsetQ, filename, anchor_gps, anchor_enu=[[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]]):
        super().__init__(offsetQ, filename, anchor_gps)
        self.anchorPosition_enu = anchor_enu

    def metadata_initialize(self, anchorPosition_gps):
        # self.anchorPosition_enu = self.anchorPosition_enu
        print('UWBSimulate_enuGPS:metadata_initialize')


if __name__ == '__main__':
    uwbmanager = UWBSimulate(os.path.dirname(
        __file__)+'/uwbData/UWB_dis_18_49_17.json')
    print(uwbmanager)
    uwbmanager.start()
    while True:
        time.sleep(1.)
        print(uwbmanager.distanceData)
        # except KeyboardInterrupt:
        #     # print(e)
        #     uwbmanager.stop()
        #     print('uwbmanager.stop()')

    # readUwb()
    # uwbSimulate()
    # calOffset(distanceQ)
    # start_time = time.time
    # myTime = datetime.strptime('18:05:36','%H:%M:%S').time()
    # newTime = datetime.strptime('18:20:36','%H:%M:%S').time()
    # c = newTime - myTime
    # print(newTime - myTime)
