import os

# check_large_cone checks how many large cone instances are in the images and files remaining how many files remain after deletint pictures without cones
check_large_cone = 0
files_remaining = 11569

if __name__ == '__main__':
    # define base directory where the dataset, images and labels are stored
    base_directory = '/home/ubuntu/Cone_Dataset/yolo_fsoco_bounding_boxes_train/'
    directory_labels = base_directory + 'labels'
    directory_picture = base_directory + 'images'

    # for loop runs through all files in directory
    for filename in os.listdir(directory_labels):

        # defines full file path for current file in for loop
        file_label = os.path.join(directory_labels, filename)
        file_image = os.path.join(directory_picture, filename[:-4])


        # opens file in read and write mode
        with open(file_label, "r+") as f:
            d = f.readlines()
            f.seek(0)
            # if class three is detected as first element in the column it is kept. If not it is deleted
            for i in d:
                if i[0] == "3":
                    f.write(i)
                    check_large_cone += 1
            f.truncate()

        # if the txt file is empty it is deleted with the corresponding image
        if os.path.getsize(file_label) == 0:
            files_remaining -= 1
            os.remove(file_label)
            os.remove(file_image)

print(check_large_cone)
print(files_remaining)