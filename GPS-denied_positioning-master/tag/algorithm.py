import serial
import numpy as np
import codecs
import binascii
import random
import threading
import collections
from datetime import datetime
#from loc_method import lsq_method, two_stage, costfun_method
import json
import time
#import crc16
from scipy import optimize
import sys, collections, time
from scipy.optimize import lsq_linear, root, minimize
# from tqdm import tqdm, trange
import numpy.matlib 
from itertools import product
from itertools import combinations
from collections import Counter
import numpy as np
#import matplotlib.pyplot as plt 
#from mpl_toolkits.mplot3d import Axes3D
import heapq
from sympy import *
import pandas as pd
import pymap3d as pm
import cmath

delay = 0.0
delay_two_stage = 0.0

from sympy import symbols, expand
from sympy.parsing.sympy_parser import parse_expr
from collections import OrderedDict

def extract_coefficients(expression):
    z = symbols('z')
    expanded_expr = expand((expression))
    coefficients = Poly(expanded_expr, z).all_coeffs()

    return coefficients

def Cardano(a,b,c,d):
    
    complex_num = (-1+cmath.sqrt(3)*1j)/2
    complex_num_2 = complex_num**2
    z0=b/3/a
    a2,b2 = a*a,b*b
    p=-b2/3/a2 +c/a
    q=(b/27*(2*b2/a2-9*c/a)+d)/a
    D=-4*p*p*p-27*q*q
    r=cmath.sqrt(-D/27+0j)
    u=((-q-r)/2)**0.33333333333333333333333
    v=((-q+r)/2)**0.33333333333333333333333

    
    z_candidate = [u+v-z0, u*complex_num + v *complex_num_2-z0, u*complex_num_2 + v*complex_num-z0]
    return z_candidate
    
def swapEndianness(hexstring):
	ba = bytearray.fromhex(hexstring)
	ba.reverse()
	return ba.hex()

# def crccitt(byte_seq):
#     crc = crc16.crc16xmodem(byte_seq, 0xffff)
#     return '{:07X}'.format(crc & 0xffff)
def lsq_method(distances_to_anchors, anchor_positions, u):
    # distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    # if not np.all(distances_to_anchors):
    #     raise ValueError('Bad uwb connection. distances_to_anchors must never be zero. ' + str(distances_to_anchors))
    anchor_offset = anchor_positions[0]
    anchor_positions = anchor_positions[1:] - anchor_offset
    K = np.sum(np.square(anchor_positions), axis=1)   #ax=1 列加
    squared_distances_to_anchors = np.square(distances_to_anchors)
    squared_distances_to_anchors = (squared_distances_to_anchors - squared_distances_to_anchors[0])[1:]
    b = (K - squared_distances_to_anchors) / 2.
    det = u.T @ b
    #res = lsq_linear(anchor_positions, b, lsmr_tol='auto', verbose=0)
    #res = np.dot(np.dot(np.linalg.inv(np.dot(anchor_positions.T, anchor_positions)),(anchor_positions.T)), b)
    res = np.linalg.lstsq(anchor_positions, b, rcond=None)[0]
    return res + anchor_offset, det, b

def two_stage(distances_to_anchors, anchor_positions, u):
    tag_pos, det, b = lsq_method(distances_to_anchors, anchor_positions, u)
    # print(tag_pos)
    z = symbols('z') #, real = True
    f = symbols('f', cls = Function)
    f = 0
    for i in range(anchor_positions.shape[0]):
        delta = distances_to_anchors[i]**2 - ((tag_pos[0]- anchor_positions[i][0])**2 + (tag_pos[1]- anchor_positions[i][1])**2)
        f += 4 * ((z - anchor_positions[i][2]) ** 3 - delta*((z)-anchor_positions[i][2]))
    
    coeff = extract_coefficients(f)
    z_candidate = Cardano(coeff[0], coeff[1], coeff[2], coeff[3])

    z_candidate = np.array([complex(item) for item in z_candidate])
    z_candidate = np.round(np.array([abs(z_candidate[0]), abs(z_candidate[1]), abs(z_candidate[2])]),5)
    z_candidate_max = max(z_candidate)

    result = list()
    check_ls = list()
    
    two_ans = np.array([tag_pos[0], tag_pos[1], z_candidate_max])
    result.append(two_ans)
    
    return np.array(result).astype(np.float32), det, b    


def _main():
    z = 1.10
    anchor_positions = [[0, 0, z],
                        [24.32, 0, z],
                        [24.32, 5.63, z],
                        [0, 5.63, z]]

    class Ref_point():
        def __init__(self, ref_point_ENU, lat, lon):
            # ENU
            self.x = ref_point_ENU[0]
            self.y = ref_point_ENU[1]
            self.z = ref_point_ENU[2]
            self.ENU = ref_point_ENU
            # lat, lon
            self.lat = lat
            self.lon = lon
            self.height = ref_point_ENU[2]
        def print_para(self):
            print('Ref point info:')
            print('ENU x: {x},  ENU y: {y},  ENU z: {z}'.format(x = self.x, y = self.y, z = self.z))
            print('lat  : {lat},  lon  : {lon}'.format(lat = self.lat, lon = self.lon))


    ref_point = Ref_point(anchor_positions[1], 25, 121)
    ref_point.print_para()
    

        
    anchor_positions = np.array(anchor_positions)
    anchor_offset = anchor_positions[0] 
    A = anchor_positions[1:] - anchor_offset
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    global delay_two_stage
    two_stage_result_ls = []
    counter = 0
    t = time.time()
    av = 0
    dis_to_tag_ls = []
    total_count = 10
    dis_to_tag = np.array([2.12,2.12,2.12,2.12])
    t_start = time.time()
    two_stage_result, det, b = two_stage(dis_to_tag, anchor_positions, u)
    t_end = time.time()
    offset = two_stage_result[0] - ref_point.ENU
    
    print('spend time:',t_end-t_start)
    print('2-stage result:',two_stage_result[0])
    print('offset:',offset)
    lat, lon, height = pm.enu2geodetic(
            offset[0], offset[1], offset[2], ref_point.lat, ref_point.lon, ref_point.height)
    print('lon, lat, height:',lon, lat, height)

    # # f.close()
    



if __name__ == '__main__':  
    _main()
