import serial
import numpy as np
import binascii
import collections
import time
import sys, collections, time
import numpy as np
from sympy import *


def swapEndianness(hexstring):
	ba = bytearray.fromhex(hexstring)
	ba.reverse()
	return ba.hex()


def UWB_dis(COM_PORT):
  
    BAUD_RATES = 57600    
    ser_UWB = serial.Serial(COM_PORT, BAUD_RATES)
    rx = ser_UWB.read(264)
    print('Enter UWB_dis')
    # print('rx:',rx)
    # print('hexlify:',binascii.hexlify(rx))
    # print('utf-8:',binascii.hexlify(rx).decode('utf-8'))

    
    dis_queue = collections.deque(maxlen = 1)
    t = time.time()
    rx = ser_UWB.read(264)
    rx = binascii.hexlify(rx).decode('utf-8')
    good_size =0
    #print('rx', rx)
    # if len(rx) == 5280:
        # print("yee")
    global dis_1
    global dis_2
    global dis_3
    global dis_4
    global dis
    
    if( rx != ' ' and rx.find('0241222000000000') >= 0 and rx.find('0241222000000000') <= (len(rx)-24)):
        
        dis_1_index = rx.find('0241222000000000') 
        dis_1 = rx[dis_1_index + 16 : dis_1_index + 24]
        dis_time1 = rx[dis_1_index - 18 : dis_1_index - 16]
        dis_1 = swapEndianness(dis_1)
        if dis_1 != "":
            dis_1 = int(dis_1,16)
            dis_1 = dis_1/100 
            dis_1 = round(float(dis_1),2)
        else:
           dis_1 = 0
    else:
        dis_1 = 0
    
    if( rx != ' ' and rx.find('0341222000000000') >= 0 and rx.find('0341222000000000') <= (len(rx)-24)):
        
        dis_2_index = rx.find('0341222000000000')
        dis_2 = rx[dis_2_index + 16 : dis_2_index + 24]
        dis_time2 = rx[dis_2_index - 18 : dis_2_index - 16]
        dis_2 = swapEndianness(dis_2)
        
        if dis_2 != "":
            dis_2 = int(dis_2,16)
            dis_2 = round(dis_2/100,2)
        else:
           dis_2 = 0
    else:
        dis_2 = 0
    
    if( rx != ' ' and rx.find('0441222000000000') >= 0 and rx.find('0441222000000000') <= (len(rx)-24)):
        
        dis_3_index = rx.find('0441222000000000')
        dis_3 = rx[dis_3_index + 16 : dis_3_index + 24]
        dis_time3 = rx[dis_3_index - 18 : dis_3_index - 16]
        dis_3 = swapEndianness(dis_3)
        
        if dis_3 != "":
            dis_3 = int(dis_3,16)
            dis_3 = round(dis_3/100,2)
        else:
           dis_3 = 0
    else:
        dis_3 = 0
        
    if( rx != ' ' and rx.find('0541222000000000') >= 0 and rx.find('0541222000000000') <= (len(rx)-24)):
        
        dis_4_index = rx.find('0541222000000000')
        dis_4 = rx[dis_4_index + 16 : dis_4_index + 24]
        dis_time4 = rx[dis_4_index - 18 : dis_4_index - 16]
        dis_4 = swapEndianness(dis_4)
        
        if dis_4 != "":
            dis_4 = int(dis_4,16)
            dis_4 = round(dis_4/100,2)
        else :
           dis_4 = 0
    else:
        dis_4 = 0
        
    dis = np.array([dis_1, dis_2, dis_3, dis_4])
    #print("dis:",dis)
    elapsed = time.time() - t
    # print(elapsed)
    #delay += elapsed
  
    return dis
    # else:
    #     UWB_dis()

def _main():
    position = '(1.84,1.83,1.04)'
    #string_time = datetime.now().strftime("%H_%M_%S")
    # data_filename = 'UWB_two_stage' + position +'.csv'
    # data_filename2 = 'package loss' + position +'.txt'
    data_filename1 = 'dis_tag'+ position +'.csv'
    # right_time_q = collections.deque(maxlen = 1)
    z = 1.10
    anchor_positions = [[0, 0, z],
                        [24.32, 0, z],
                        [24.32, 5.63, z],
                        [0, 5.63, z]]
        
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
    while True:
        dis_to_tag = UWB_dis()
        print(counter, dis_to_tag)
        # dis_to_tag = UWB_dis()
        # dis_to_tag_ls.append(dis_to_tag)

        # dis_to_tag = np.array([2.12,2.12,2.12,2.12])

        #print('dis_to_tag',dis_to_tag)

        # rightnow_time = datetime.now().strftime("%H:%M:%S")
        # right_time_q.append(rightnow_time)
        # right_t = right_time_q.popleft()
        counter = counter + 1
        
        # if(0 not in dis_to_tag):
        #     # t_two_stage = time.time()
        #     # # two_stage_result, det, b = two_stage(dis_to_tag, anchor_positions, u)
        #     # # # two_stage_result, det, b = costfun_method(dis_to_tag, anchor_positions, u)
        #     # # # # two_stage_result[:,2] = z - abs(two_stage_result[:,2] - z)
        #     # # two_stage_result = two_stage_result.reshape(3)
        #     # # print('two_stage:', two_stage_result)
        #     # elapsed_two_stage = time.time()-t_two_stage
        #     # delay_two_stage += elapsed_two_stage
        #     av += 1
        #     print(av)
            # two_stage_result_ls.append(two_stage_result)
    # dis_to_tag_ls = np.array(dis_to_tag_ls)
    # dis_to_tag_ls = np.transpose(dis_to_tag_ls)
    # df1 = pd.DataFrame({'A6':dis_to_tag_ls[:][0].tolist(),
    #                 'A3': dis_to_tag_ls[:][1].tolist(),
    #                 'A7': dis_to_tag_ls[:][2].tolist(),
    #                 'A5':dis_to_tag_ls[:][3].tolist()})
    # df1.to_csv(data_filename1)                         
    # two_stage_result_ls = np.array(two_stage_result_ls)
    # two_stage_result_ls = np.transpose(two_stage_result_ls)
    # df = pd.DataFrame({'x':two_stage_result_ls[:][0].tolist(),
    #                 'y': two_stage_result_ls[:][1].tolist(),
    #                 'z': two_stage_result_ls[:][2].tolist()})
    # df.to_csv(data_filename)
        # with open(data_filename, 'a') as fout:
        # json.dump({'time': right_t, 'dis': dis_to_tag.tolist(), 'two_stage_result' : two_stage_result.tolist()}, fout)
        # json.dump({'time': right_t, 'dis': dis_to_tag}, fout)
    
        # with open(data_filename, 'a') as fout:
        #      json.dump({'two_stage': two_stage_result.tolist()}, fout)    
    #elapsed = time.time() - t
    # f = open(data_filename2,'w')
    # f.write('Hz:%f\n'%(av/delay))
    # f.write('valid:%f\n'%(av))
    # f.write('package loss:%f\n'%(av/counter))
    # f.write('slove time:%f\n'%(delay_two_stage/av))
    # f.close()
    


    # print('Hz',av/elapsed)
    # print("valid",av)
    # print('package loss:',av/counter)
    # print(counter)
if __name__ == '__main__':  
    _main()