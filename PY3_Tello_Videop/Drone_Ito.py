#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tello
import time
import cv2
import sys
import numpy as np

# マーカーサイズ
marker_length = 0.056 # [m]

#arucoライブラリ
aruco = cv2.aruco

#arucoマーカー辞書
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

#カメラパラメータ
camera_matrix = np.array([[639.87721705,   0.        , 330.12073612],
                            [  0.        , 643.69687408, 208.61588364],
                            [  0.        ,   0.        ,   1.        ]])
distortion_coeff = np.array([ 5.66942769e-02, -6.05774927e-01, -7.42066667e-03, -3.09571466e-04, 1.92386974e+00])


# メイン関数
def main():

    # Telloクラスを使って，droneというインスタンス(実体)を作る
    drone = tello.Tello('', 8889, command_timeout=.01)

    current_time = time.time()  # 現在時刻の保存変数
    pre_time = current_time     # 5秒ごとの'command'送信のための時刻変数

    time.sleep(0.5)     # 通信が安定するまでちょっと待つ
    
    drone.takeoff()
    
    M = 0
    
    
    #Ctrl+cが押されるまでループ
    try:
        while True:

            # (A)画像取得
            frame = drone.read()
            if frame is None or frame.size == 0:
                continue

            image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            small_image = cv2.resize(image, dsize=(480,360) )
            corners, ids, rejectedImgPoints = aruco.detectMarkers(small_image, dictionary)
            aruco.drawDetectedMarkers(small_image, corners, ids, (0,255,0))
            
            
            
            
            #離陸と高さ調整
            if M == 0:
                #gomi = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
                #ids = np.delete(ids , gomi)
            
                if ids == 1:
                    drone.move_up(0.3)
                    #drone.move_up(0.1)
                    #time.sleep(3.0)
                    M = M + 1
                   
                else:
                    drone.move_up(0.3)
                    #drone.move_up(0.2)
                    #time.sleep(2.0)
                    
                key = cv2.waitKey(1)
                
            
            #マーカー距離計算
            for i, corner in enumerate(corners):
                        
                        # rvec -> rotation vector, tvec -> translation vector
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distortion_coeff)
                        # < rodoriguesからeuluerへの変換 >
                        # 不要なaxisを除去
                        tvec = np.squeeze(tvec)
                        rvec = np.squeeze(rvec)
                        # 回転ベクトルからrodoriguesへ変換
                        rvec_matrix = cv2.Rodrigues(rvec)
                        rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
                        # 並進ベクトルの転置
                        transpose_tvec = tvec[np.newaxis, :].T
                        # 合成
                        proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                        # オイラー角への変換
                        euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]

                        print("x : " + str(tvec[0]))
                        print("y : " + str(tvec[1]))
                        print("z : " + str(tvec[2]))
                        print("roll : " + str(euler_angle[0]))
                        print("pitch: " + str(euler_angle[1]))
                        print("yaw  : " + str(euler_angle[2]))
                        # 可視化
                        draw_pole_length = marker_length/2 # 現実での長さ[m]
                        aruco.drawAxis(small_image, camera_matrix, distortion_coeff, rvec, tvec, draw_pole_length)
                        
            #No2 マーカーへの移動
            if M == 1:
                #gomi = [1,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
                #ids = np.delete(ids , gomi)
                print("M",M)
                print("ids",ids)
                
                
                if not (ids == None):
                    ids_true = ids[ids-2 == 0]
                    
                    if ids_true == 2:
                        #if ids == 2:
                        if tvec[2] > 0.4:
                            for i in range(len(ids)):
                                c = corners[i][0]
                                x1, x2, x3, x4 = c[:, 0]
                                y1, y2, y3, y4 = c[:, 1]
                                        
                                print(f"id={ids[i]}")
                                print("X座標", x1, x2, x3, x4)
                                print("Y座標", y1, y2, y3, y4)
                                print("中心座標", c[:, 0].mean(), c[:, 1].mean())
                                if c[:, 0].mean() < 140:
                                    drone.rotate_ccw(10)
                                if c[:, 0].mean() > 340:
                                    drone.rotate_cw(10)
                                if c[:, 1].mean() < 140:
                                    drone.move_up(0.1)
                                if c[:, 1].mean() > 340:
                                    drone.move_down(0.1)
                            drone.move_forward(0.3)
                            time.sleep(2.0)
                        else:
                            M = M + 1
                            time.sleep(2.0)
                            
                    else:
                        drone.rotate_ccw(20)
                        time.sleep(2.0)
                else:
                    drone.rotate_ccw(20)
                    time.sleep(2.0)
                         # 検知したidの4点取得
                        
                        
            if M == 2:
                print("M",M)
                print("ids",ids)
                
                
                if not (ids == None):
                    ids_true = ids[ids-1 == 0]
                    
                    if ids_true == 1:
                        #if ids == 2:
                        if tvec[2] > 0.4:
                            for i in range(len(ids)):
                                c = corners[i][0]
                                x1, x2, x3, x4 = c[:, 0]
                                y1, y2, y3, y4 = c[:, 1]
                                        
                                print(f"id={ids[i]}")
                                print("X座標", x1, x2, x3, x4)
                                print("Y座標", y1, y2, y3, y4)
                                print("中心座標", c[:, 0].mean(), c[:, 1].mean())
                                if c[:, 0].mean() < 140:
                                    drone.rotate_ccw(10)
                                if c[:, 0].mean() > 340:
                                    drone.rotate_cw(10)
                                if c[:, 1].mean() < 140:
                                    drone.move_up(0.1)
                                if c[:, 1].mean() > 340:
                                    drone.move_down(0.1)
                            drone.move_forward(0.3)
                            time.sleep(2.0)
                        else:
                            drone.land()
                            time.sleep(2.0)
                            brake
                    else:
                        drone.rotate_ccw(20)
                        time.sleep(2.0)
                                
                else:
                    drone.rotate_ccw(20)
                    time.sleep(2.0)
                
                key = cv2.waitKey(1)
                
            cv2.imshow('OpenCV Window', small_image)
                
            current_time = time.time()  # 現在時刻を取得
            if current_time - pre_time > 5.0 :  # 前回時刻から5秒以上経過しているか？
                drone.send_command('command')   # 'command'送信
                pre_time = current_time         # 前回時刻を更新
    except( KeyboardInterrupt, SystemExit):
        drone.land()# Ctrl+cが押されたら離脱
        print( "SIGINTを検知" )
    # telloクラスを削除
    del drone
if __name__ == "__main__":
    main()    # メイン関数を実行



