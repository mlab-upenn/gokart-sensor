import os

old_class_number = "3"
new_class_number = "0"
base_directory = '/home/ubuntu/Cone_Dataset/labels'

if __name__ == '__main__':


    for filename in os.listdir(base_directory):

        file_label = os.path.join(base_directory, filename)

        with open(file_label, "r+") as f:
            d = f.readlines()
            f.seek(0)
            # if class three is detected as first element in the column it is kept. If not it is deleted
            for i in d:
                if i[0] == old_class_number:
                    i = new_class_number + i[1:]
                    f.write(i)
                else:
                    f.write(i)
            f.truncate()