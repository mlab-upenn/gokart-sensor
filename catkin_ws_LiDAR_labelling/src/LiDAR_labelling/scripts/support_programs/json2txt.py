#!/usr/bin/env python3

import os
import json

datapath_json = "/home/felix/gokart-sensor/01_LiDAR_Dataset/json"
datapath_labels = "/home/felix/gokart-sensor/01_LiDAR_Dataset/txt"

for filename in os.listdir(datapath_json):
    # open json file
    f_json = open(datapath_json + "/" + filename)

    # read data from json as dictionary
    json_data = json.load(f_json)

    # read json filename and save for corresponding txt file
    txt_filename = json_data.get('filename')[:-3] + "txt"
    #
    f_txt = open(datapath_labels + "/" + txt_filename, "w")
    f_txt.close()
    f_txt = open(datapath_labels + "/" + txt_filename, "r+")

    # iterate through every cone instance
    for i in json_data['objects']:
        center_point_x = i.get('centroid').get('x')
        center_point_y = i.get('centroid').get('y')
        center_point_z = i.get('centroid').get('z')

        dx = i.get('dimensions').get('length')
        dy = i.get('dimensions').get('width')
        dz = i.get('dimensions').get('height')

        alpha = i.get('rotations').get('z')

        # write data to txt file
        f_txt.write(str(center_point_x) + " " + str(center_point_y) + " " + str(center_point_z) + " " + str(dx) + " " + str(dy) + " " + str(dz) + " " + str(alpha) + " Vehicle\n")
        f_txt.truncate()


    # close json file
    f_json.close

    # close txt file
    f_txt.close()