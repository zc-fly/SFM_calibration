# make use colmap act camera calibrations, tansform colmap camera model 2 json format
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

import colmap_model_trans as colTransformer
from json_formatter import formatter


def simple_camera():
    canNums = 4 #cameras nums
    model_path = "/home/chen/PROJECT_code/sfm_Calibration/colmap_calib/simple_camera"
    # transform camera model file from .bin 2 .txt
    cameras, images, points3D = colTransformer.read_model(path=model_path, ext=".bin")
    colTransformer.write_model(cameras, images, points3D, path=model_path, ext=".txt")

    # read txt camera model
    intTXT = model_path+"/cameras.txt"
    outTXT = model_path + "/images.txt"

    intInfo_temp = []
    outInfo_temp = []
    with open(intTXT, "r") as f:
        for line in f.readlines():
            data = line.split()
            intInfo_temp.append(np.array(data))

    with open(outTXT, "r") as f:
        for line in f.readlines():
            data = line.split()
            outInfo_temp.append(np.array(data))

    intInfo = []
    outInfo = []
    for i in range(canNums):
        intInfo.append(intInfo_temp[i+3])
        outInfo.append(outInfo_temp[2*i + 4])
    intInfo.sort(key=lambda x: x[0])
    outInfo.sort(key=lambda x: x[8])

    # intrInfo = [[1434.6038015152826, 1439.6380466207102, 960.0, 540.0],[1423.5728234312821, 1395.918093756117,960.0 ,540.0],[1429.9628827626025 ,1397.8235613436507 ,960.0, 540.0],[1428.6341647288884 ,1412.2706755123695 ,960.0 ,540.0]] #fx, fy, cx, cy
    # distortion = [[-0.17901642646329802, 0.26340834395023066 ,0.014989692146156069 ,-0.001144159328791238,0],[-0.1460041542028574 ,0.21314508953372976 ,0.002162659081013709 ,-0.0026530875077544905,0],[-0.1176501133086549 ,0.1865826946351153 ,-0.0028618674372962874 ,0.0007477917164757779,0],[-0.11893486360272838 ,0.18158820539854909 ,-0.007374707078012142 ,0.0036906283792325456,0]] #k1,k2,p1,p1
    # quat = [[0.8220589191582908, -0.5454271097703649 ,-0.01500346496976934 ,-0.1627983335257157],[0.9330012380207606 ,-0.3345213440259579 ,-0.03743514050817025 ,-0.1272901036920322],[0.9813385560733757, -0.18230062482464351 ,-0.008675517191047954 ,-0.06054631247098709],[1.0 ,0.0 ,0.0 ,0.0]]
    # trans_T = [[-2.021740197018474 ,-10.190800382464289 ,4.028090381311065],[-1.1382463177461257 ,-4.667282459818665 ,0.3114803784780712],[-1.4560389756788699, 0.00951580725561791 ,1.087280530434395],[-1.6029689969043928 ,4.518489311145171 ,1.7234968861237872]]

    #transform calibrate result from colmap 2 json
    json_list = formatter(canNums)
    for i in range(canNums):

        r = R.from_quat((outInfo[i][1:5]).astype(float))
        rotation_matrix = r.as_matrix()
        extMatrix = np.zeros((4, 4))
        extMatrix[0:3, 0:3] = rotation_matrix
        extMatrix[:3, 3] = np.array((outInfo[i][5:8]).astype(float)).flatten()
        extMatrix[3, 3] = 1

        intMatrix = [[(intInfo[i][4]).astype(float), 0, (intInfo[i][5]).astype(float)], [0, (intInfo[i][4]).astype(float), intInfo[i][6].astype(float)], [0, 0, 1]]
        distranstorm = np.zeros((1,5))
        distranstorm[0,0] = (intInfo[i][7]).astype(float)

        var = 'Camera' + str(i+1)
        json_list[var]['CameraMatrix'] = np.array(extMatrix).flatten().tolist()
        json_list[var]['CameraIntrinsic'] = np.array(intMatrix).flatten().tolist()
        json_list[var]['CameraNumber'] = i+1
        json_list[var]['DistCoeff'] = np.array(distranstorm).flatten().tolist()
        json_list[var]['FrameHeight'] =intInfo[i][2]
        json_list[var]['FrameWidth'] =intInfo[i][3]
        json_list[var]['fps'] = 60

    with open("camera_model.json", "w") as outfile:
        json.dump(json_list, outfile)

def opencv_camera():
    canNums = 4 #cameras nums
    model_path = "/home/chen/PROJECT_code/sfm_Calibration/colmap_calib/opencv_camera"
    # transform camera model file from .bin 2 .txt
    # cameras, images, points3D = colTransformer.read_model(path=model_path, ext=".bin")
    # colTransformer.write_model(cameras, images, points3D, path=model_path, ext=".txt")

    # read txt camera model
    intTXT = model_path+"/cameras.txt"
    outTXT = model_path + "/images.txt"

    intInfo_temp = []
    outInfo_temp = []
    with open(intTXT, "r") as f:
        for line in f.readlines():
            data = line.split()
            intInfo_temp.append(np.array(data))

    with open(outTXT, "r") as f:
        for line in f.readlines():
            data = line.split()
            outInfo_temp.append(np.array(data))

    intInfo = []
    outInfo = []
    for i in range(canNums):
        intInfo.append(intInfo_temp[i+3])
        outInfo.append(outInfo_temp[2*i + 4])
    intInfo.sort(key=lambda x: x[0])
    outInfo.sort(key=lambda x: x[8])

    # intrInfo = [[1434.6038015152826, 1439.6380466207102, 960.0, 540.0],[1423.5728234312821, 1395.918093756117,960.0 ,540.0],[1429.9628827626025 ,1397.8235613436507 ,960.0, 540.0],[1428.6341647288884 ,1412.2706755123695 ,960.0 ,540.0]] #fx, fy, cx, cy
    # distortion = [[-0.17901642646329802, 0.26340834395023066 ,0.014989692146156069 ,-0.001144159328791238,0],[-0.1460041542028574 ,0.21314508953372976 ,0.002162659081013709 ,-0.0026530875077544905,0],[-0.1176501133086549 ,0.1865826946351153 ,-0.0028618674372962874 ,0.0007477917164757779,0],[-0.11893486360272838 ,0.18158820539854909 ,-0.007374707078012142 ,0.0036906283792325456,0]] #k1,k2,p1,p1
    # quat = [[0.8220589191582908, -0.5454271097703649 ,-0.01500346496976934 ,-0.1627983335257157],[0.9330012380207606 ,-0.3345213440259579 ,-0.03743514050817025 ,-0.1272901036920322],[0.9813385560733757, -0.18230062482464351 ,-0.008675517191047954 ,-0.06054631247098709],[1.0 ,0.0 ,0.0 ,0.0]]
    # trans_T = [[-2.021740197018474 ,-10.190800382464289 ,4.028090381311065],[-1.1382463177461257 ,-4.667282459818665 ,0.3114803784780712],[-1.4560389756788699, 0.00951580725561791 ,1.087280530434395],[-1.6029689969043928 ,4.518489311145171 ,1.7234968861237872]]

    #transform calibrate result from colmap 2 json


    json_list = formatter(canNums)
    for i in range(canNums):

        r = R.from_quat(outInfo[i][1:5])
        rotation_matrix = r.as_matrix()
        extMatrix = np.zeros((4, 4))
        extMatrix[0:3, 0:3] = rotation_matrix
        extMatrix[:3, 3] = np.array(outInfo[i][5:8]).astype(float).flatten()
        extMatrix[3, 3] = 1

        intMatrix = [[np.array(intInfo[i][4]).astype(float), 0, np.array(intInfo[i][6]).astype(float)], [0, np.array(intInfo[i][5]).astype(float), np.array(intInfo[i][7]).astype(float)], [0, 0, 1]]
        distranstorm = np.zeros((1,5))
        distranstorm[0,0:4] = np.array(intInfo[i][8:12]).astype(float)

        var = 'Camera' + str(i+1)
        json_list[var]['CameraMatrix'] = np.array(extMatrix).flatten().tolist()
        json_list[var]['CameraIntrinsic'] = np.array(intMatrix).flatten().tolist()
        json_list[var]['CameraNumber'] = i+1
        json_list[var]['DistCoeff'] = np.array(distranstorm).flatten().tolist()
        json_list[var]['FrameHeight'] =intInfo[i][2]
        json_list[var]['FrameWidth'] =intInfo[i][3]
        json_list[var]['fps'] = 60

    with open("camera_model.json", "w") as outfile:
        json.dump(json_list, outfile)

if __name__ == "__main__":
    # simple_camera()
    opencv_camera()