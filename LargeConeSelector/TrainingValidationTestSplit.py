import splitfolders
import os

base_directory = '/home/ubuntu/Cone_Dataset/Full_Set/images'
output_directory = "/home/ubuntu/Cone_Dataset/Full_Set/split_dataset"

directory_training = output_directory + '/train/cones'
directory_val = output_directory + '/val/cones'
directory_test = output_directory + '/test/cones'

if __name__ == '__main__':
    print("Starting process")
    # splitfolders.ratio('/home/ubuntu/Cone_Dataset/Full_Set/images', output="/home/ubuntu/Cone_Dataset/Full_Set/split_dataset", seed=1337, ratio=(.8, 0.1, 0.1))

    training_txt = open(output_directory + "/train.txt", "a")
    for filename in os.listdir(directory_training):
        training_txt.write(filename + "\n")
    training_txt.close()

    val_txt = open(output_directory + "/valid.txt", "a")
    for filename in os.listdir(directory_val):
        val_txt.write(filename + "\n")
    val_txt.close()

    test_txt = open(output_directory + "/test.txt", "a")
    for filename in os.listdir(directory_test):
        test_txt.write(filename + "\n")
    test_txt.close()
