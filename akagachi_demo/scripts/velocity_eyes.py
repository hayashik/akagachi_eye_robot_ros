#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np

class Veye(object):

    def __init__(self):
        debug = True

        self.servos_command = []
        self.t0 = time.time()                                            #計算開始時刻

    def blink_max_vel(self, blink_anp=0.0, blink_flug=False):
        '''
        振幅を指定して，閉じる時の最大速度を算出する

        @param blinkAnp 瞬きの振幅 (degree)
        @return 値

        according to "Eyelid movements. Mechanisms and normal data"
        '''
        max_vel = 0

        if(blink_flug):
            max_vel = 29.2 * blink_anp - 35.9
        elif(not blink_flug):
            max_vel = 13.5 * blink_anp - 5.87

        return max_vel  #deg/sec

    def blink_duration(self, blink_anp=0.0, blink_flug=False):
        '''
        振幅を指定して，閉じる時の時間を算出する(sec)

        @param blinkAnp 瞬きの振幅 （degree）
        @return 値

        Eyelid movements. Mechanisms and normal dataより
        '''

        max_d = 0

        if(blink_flug):
            max_d = 36.3 + 1.4 * blink_anp - 0.016 * math.pow(blink_anp, 2)
        elif(not blink_flug):
            max_d = 87.9 + 4.3 * blink_anp - 0.047 * math.pow(blink_anp, 2)

        return max_d

    def differentiated_gaussian_function_vel(self, sigma, calc_value, con, max_vel):
        '''
        変数とシグマを入力することで，ガウス関数の一次微分の解から得た速度を返す

        @param sigma シグマ値
        @param 変数
        @param ガウス関数一次微分調整用の最大値
        @param 最大速度
        @return 速度
        '''

        #calc_tmp1=0.0, calc_tmp2 =0.0, calc =0.0

        #calc_tmp1 = -1 * calc_value  / (math.sqrt(2 * math.pi) * math.pow(sigma, 3))
        calc_tmp1 = -1 * calc_value  / math.pow(sigma, 2)
        calc_tmp2 = math.exp((-1) * math.pow(calc_value, 2) / (2 * math.pow(sigma, 2)))

        calc = abs(calc_tmp1 * calc_tmp2 * max_vel )

        #calc = max_vel if calc > max_vel else calc

        return calc

    def differentiated_gaussian_function_angle(self, sigma, calc_value, con, target_angl):
        '''
        変数とシグマを入力することで，ガウス関数の一次微分の解から得た速度を返す

        @param sigma シグマ値
        @param 変数
        @param ガウス関数一次微分調整用の最大値
        @param 最大速度
        @return 速度
         '''

        calc_tmp1 = 1 / (math.sqrt(2 * math.pi) * sigma)
        calc_tmp2 = math.exp((-1) * math.pow(calc_value, 2) / (2 * math.pow(sigma, 2)))

        calc = abs(calc_tmp1 * calc_tmp2 * target_angl / con)

        return calc

    def differentiated_lainer_function_angle(self, calc_value, max_du, max_vel):
        '''
        変数とシグマを入力することで，ガウス関数の一次微分の解から得た速度を返す

        @param sigma シグマ値
        @param 変数
        @param ガウス関数一次微分調整用の最大値
        @param 最大速度
        @return 速度
         '''

        calc = max_vel/(max_du*2/3)*calc_value if (calc_value <= max_du*2/3) else max_vel -1 * max_vel/(max_du*2/3)*(calc_value-max_du*2/3)
        calc = 1 if (calc < 0) else calc

        return calc

'''
    def natural_blink_function(self, blink_deg=np.zeros(4)):
'''
        #ガウス関数に沿って瞬きを行う関数
        #@param ()
        #* @param blink_deg[0] 左瞼の閉じ角度
        #* @param blink_deg[1] 左瞼の開き角度（通常原点）
        #* @param blink_deg[2] 右瞼の閉じ角度
        #* @param blink_deg[3] 右瞼の開き角度（通常原点）
        #* @return 値
'''

        #sigma_down = 15 sigma_up = 50
        sigma = [15, 50]

        calc_vel = np.zeros(2)
        order_t = np.zeros(2)
        # calc_vell = 0.0

        calc_time = 0.0
        present_time = 0.0

        time_tmp = 0;				#[deg/msec]

        target_angl = abs(blink_deg[0] - blink_deg[1]);					    #[deg]
        #pre_angl = blink_deg[1]    pre_angr = blink_deg[3]
        pre_ang = [blink_deg[1], blink_deg[3]]

        down_du = self.down_blink_duration(target_angl*2);          #[msec]
        down_max_vell = self.down_blink_max_vel(target_angl);       #[deg/msec]

        #double targetAngR = Math.abs(rsAp - rOrg);					//[deg]

        down_time_interval = 0.0
        t0 = time.time()

        present_time = float(time.time()  - t0)		#現在時間

        while(present_time <= down_du):

            if(present_time != 0 and time_tmp != present_time):
                calc_time = present_time - down_du
                down_time_interval = down_max_vell * (present_time - time_tmp)

                #まぶたを閉じる時のガウス関数による速度計算
                calc_vel[0] = self.differentiated_gaussian_function_vel(sigma[0], calc_time, 0.0010754254423073, down_max_vell)

                if(calc_vel[0] > down_max_vell):
                    calc_vel[0] = down_max_vell

                if(pre_ang[0] + down_time_interval >= blink_deg[0]):
                    pre_ang[0] = blink_deg[0]
                else: pre_ang[0] += down_time_interval

                if(pre_ang[1] + down_time_interval >= blink_deg[2]):
                    pre_ang[1] = blink_deg[2]
                else: pre_ang[1] += down_time_interval

                order_t[0] = abs(pre_ang[0]/calc_vel[0])
                order_t[1] = abs(pre_ang[1]/calc_vel[1])

                #servo.move(1, (int)pre_angl * 10, (int)(orderTL/10))
                #servo.move(6, (int)preAngR * 10, (int)(orderTR/10))

                print("sAp = " + int(blink_deg[0]*10) + ", orderTime = " + order_t[0] + ", presetAngle = " + pre_ang[0] + 
                                ", downDuration = " + down_du)
                print("calcTime = " + int(calc_time) + ", presentTime = " + int(present_time) + ", calcVel = " + calc_vel[0] + 
                                ", downMaxVel = " + down_max_vell)

                return int(pre_ang[0]), int(order_t[0]/10), int(pre_ang[1]), int(order_t[1]/10)

            time_tmp = present_time
            present_time = float(time.time() - t0)

        target_angl = abs(blink_deg[0] - blink_deg[1])                  #deg
        up_duration = self.up_blink_duration(target_angl*2)      #[msec]
        up_max_vel = self.up_blink_max_vel(target_angl)            #[deg/msec]

        up_time_interval = up_max_vel * 10

        present_time = time.time() - t0
        pre_ang[0] = blink_deg[0]
        pre_ang[1] = blink_deg[2]

        while (present_time < up_duration + down_du and present_time >= down_du):

            if(present_time != 0 and time_tmp != present_time):

                calc_time = present_time - down_du
                up_time_interval = up_max_vel * (present_time - time_tmp)

                #//まぶたを上げる時のガウス関数による速度計算
                calc_vel[0] = self.differentiated_gaussian_function_vel(sigma[1], calc_time, 0.0000967882898076574, up_max_vel)
                calc_vel[1] = self.differentiated_gaussian_function_vel(sigma[1], calc_time, 0.0000967882898076574, up_max_vel)

                if(calc_vel[0] >= up_max_vel): 
                    calc_vel[0] = up_max_vel

                if(pre_ang[0] - up_time_interval <= blink_deg[1]): 
                    pre_ang[0] = blink_deg[1]
                else: pre_ang[0] -= up_time_interval

                if(pre_ang[1] - up_time_interval <= blink_deg[3]): 
                    pre_ang[1] = blink_deg[3]
                else: pre_ang[1] -= up_time_interval

                order_t[0] = abs((blink_deg[0]-pre_ang[0])/calc_vel[0])
                order_t[1] = abs((blink_deg[2]-pre_ang[1])/calc_vel[1])

                #servo.move(1, (int)pre_angl * 10, (int)(orderTL/10))
                #servo.move(6, (int)pre_angr * 10, (int)(orderTR/10))

                print("org = " + int(blink_deg[1]*10) + ", orderTime = " + order_t[0]/10 + ", presetAngle = " + pre_ang[0] +  
                                ", upDuration = " + up_duration)
                print("calcTime = ", int(calc_time), ", presentTime = ", int(present_time), ", calcVel = ", calc_vel[0], 
                                ", upMaxVel = ", up_max_vel)

                return int(pre_ang[0]), int(order_t[0]/10), int(pre_ang[1]), int(order_t[1]/10)

            time_tmp = present_time
            present_time = float(time.time() - t0)
'''