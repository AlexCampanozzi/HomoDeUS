import os
import uuid
import cv2
import yaml
import time
import numpy as np
from scipy import ndimage
from progress.bar import Bar


ROTATION_RANGE = [-45, 45]
LOW_GREEN = [76, 113, 66]
UPPER_GREEN = [91, 255, 255]
N_PICTURES = 10000
SCALING_LIMITS = [0.2, 1.]

def get_filter(object_class):

    apple_filter = [[71, 42, 0], [120, 255, 255]]
    banana_filter = [[71, 42, 0], [120, 255, 255]]
    cheezit_filter = [[68, 76, 61], [84, 255, 255]]
    fork_filter = [[65, 241, 0], [114, 255, 255]]
    knife_filter = [[56, 163, 0], [133, 255, 255]]
    mug_filter = [[71, 42, 0], [120, 255, 255]]
    plate_filter = [[71, 42, 0], [120, 255, 255]]
    pringles_filter = [[76, 243, 0], [123, 255, 255]]
    default_filter = [[71, 42, 0], [120, 255, 255]]

    if object_class == "apple":
        return apple_filter

    elif object_class == "banana":
        return banana_filter

    elif object_class == "cheezit":
        return cheezit_filter

    elif object_class == "fork":
        return fork_filter

    elif object_class == "knife":
        return knife_filter

    elif object_class == "mug":
        return mug_filter

    elif object_class == "plate":
        return plate_filter

    elif object_class == "pringles":
        return pringles_filter

    else:
        return default_filter


def rescale_image(image, scale):
    width = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    output = cv2.resize(image, (width, height))
    return output


def get_mask(image, low_threshold, upper_threshold):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(image_hsv, low_threshold, upper_threshold)
    # mask = cv2.blur(mask, (5, 5))
    mask = cv2.bitwise_not(mask)
    return mask


def get_object_box(mask, image=None):
    x, y, w, h = 0, 0, 0, 0
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        object_contour = max(contours, key=cv2.contourArea)

        if image is not None:
            output = image
            cv2.drawContours(output, object_contour, -1, 255, 3)

            while True:
                cv2.imshow('Image', output)

                if cv2.waitKey(25) == 27:
                    break

            cv2.destroyAllWindows()

        x, y, w, h = cv2.boundingRect(object_contour)

    return x, y, w, h


def crop_image(image, x, y, w, h):
    output = image[y:y+h, x:x+w]
    return output


def paste_object(object, mask, background):

    output = background

    # Rotation
    should_rotate = np.random.binomial(1, 0.5)

    if should_rotate:
        rotation = np.random.randint(ROTATION_RANGE[0], ROTATION_RANGE[1])
        object = ndimage.rotate(object, rotation)
        mask = ndimage.rotate(mask, rotation)

    # Position
    object_width = object.shape[1]
    object_height = object.shape[0]

    background_width = background.shape[1]
    background_height = background.shape[0]

    max_x = background_width - object_width
    max_y = background_height - object_height

    x = np.random.randint(0, max_x)
    y = np.random.randint(0, max_y)

    # Pasting image
    for i in range(object_width):
        for j in range(object_height):
            if mask[j, i]:
                output[y+j, x+i] = object[j, i]

    output = cv2.blur(output, (3, 3))

    # Normalizing bounding box
    x_norm = float(x + (object_width / 2)) / float(background_width)
    y_norm = float(y + (object_height / 2)) / float(background_height)
    w_norm = float(object_width) / float(background_width)
    h_norm = float(object_height) / float(background_height)

    return output, x_norm, y_norm, w_norm, h_norm


def save_image(name, image):
    path = "./dataset/images/" + name + ".jpg"
    cv2.imwrite(path, image)


def save_yolo_file(name, object_class, x, y, w, h, mode="w"):
    path = "./dataset/labels/" + name + ".txt"
    content = str(object_class) + " " + str(x) + " " + str(y) + " " + str(w) + " " + str(h) + "\n"

    f = open(path, mode)
    f.write(content)
    f.close()


class DatasetManager:
    def __init__(self):
        self._config_file = {}
        self._config_file["train"] = "./dataset/images/"
        self._config_file["val"] = "./dataset/images/"
        self._config_file["nc"] = 0
        self._config_file["names"] = []

    def add_class(self, name):
        self._config_file["nc"] += 1
        self._config_file["names"].append(name)

    def get_class_index(self, class_name):

        for i in range(len(self._config_file["names"])):
            name = self._config_file["names"][i]

            if name == class_name:
                return i

        return None

    def save_config(self):
        path = "./dataset/dataset.yaml"

        with open(path, 'w') as file:
            yaml.dump(self._config_file, file)


class BackgroundsManager:
    def __init__(self):

        self._backgrounds_list = []

        f = open("./backgrounds/TestImages.txt", "r")
        lines = f.readlines()

        for line in lines:
            line = line.strip()
            self._backgrounds_list.append(line)

    def get_random_background(self):

        random_index = np.random.randint(0, len(self._backgrounds_list))
        background_file = self._backgrounds_list[random_index]
        path = "./backgrounds/indoorCVPR_09/Images/" + background_file

        image = cv2.imread(path)
        return image

class AberrationDetector:
    def __init__(self):

        self._areas = {}
        self._averages = {}
        self._standard_deviations = {}


    def consider_sample(self, path, filter):

        img = cv2.imread(path)
        mask = get_mask(img, np.array(filter[0]), np.array(filter[1]))

        area = np.count_nonzero(mask)
        class_name = (path.split('/'))[-2]

        if class_name in self._areas.keys():
            self._areas[class_name] = np.append(self._areas[class_name], [area])

        else:
            self._areas[class_name] = np.array([area])

        self._averages[class_name] = np.mean(self._areas[class_name])
        self._standard_deviations[class_name] = np.std(self._areas[class_name])


    def process_raw_object(self, class_name, filter):

        directory_path = "./process/raw_objects/" + class_name + "/"
        pictures_list = os.listdir(directory_path)

        for picture in pictures_list:
            self.consider_sample(directory_path + picture, filter)


    def check_mask(self, mask, class_name, tolerance = 1.):

        area = np.count_nonzero(mask)

        quality = np.abs(area - self._averages[class_name]) / self._standard_deviations[class_name]

        if quality < tolerance:
            return True
        else:
            return False


#-------------------------------------------------------------------------------
def main():

    start_time = time.time()
    global N_PICTURES, SCALING_LIMITS

    dataset_manager = DatasetManager()
    backgrounds_manager = BackgroundsManager()
    objects_list = os.listdir("./process/raw_objects")

    for object_class in objects_list:
        directory_path = "./process/raw_objects/" + object_class + "/"
        pictures_list = os.listdir(directory_path)

        dataset_manager.add_class(object_class)

        bar = Bar('Processing (1/2) ' + object_class + ':', max=(N_PICTURES//2))
        pic_index = 0

        while pic_index < (N_PICTURES // 2):

            try:
                # Choose an object and background
                random_index = np.random.randint(0, len(pictures_list))
                random_picture = directory_path + pictures_list[random_index]

                img = cv2.imread(random_picture)
                background = backgrounds_manager.get_random_background()

                if background is None:
                    pass

                # Rescale the object
                img_w, img_h = img.shape[1], img.shape[0]
                background_w, background_h = background.shape[1], background.shape[0]

                # Get mask
                filter = get_filter(object_class)
                mask = get_mask(img, np.array(filter[0]), np.array(filter[1]))

                random_ratio = np.random.uniform(low=SCALING_LIMITS[0], high=SCALING_LIMITS[1])
                img = rescale_image(img, random_ratio)
                mask = rescale_image(mask, random_ratio)

                # mask = get_mask(img, np.array(green_filter[0]), np.array(green_filter[1]))
                box_x, box_y, box_w, box_h = get_object_box(mask)

                # Crop image
                object = crop_image(img, box_x, box_y, box_w, box_h)
                mask = crop_image(mask, box_x, box_y, box_w, box_h)

                # Combine image
                yolo_class = dataset_manager.get_class_index(object_class)
                combined_image, yolo_x, yolo_y, yolo_w, yolo_h = paste_object(object, mask, background)

                # Save everything
                filename = object_class + "_" + pictures_list[random_index].replace('.jpg', '') + '_' + str(uuid.uuid4())
                save_image(filename, combined_image)
                save_yolo_file(filename, yolo_class, yolo_x, yolo_y, yolo_w, yolo_h, mode="w")

                pic_index += 1

                bar.goto(pic_index)

            except:
                pass

    for object_class in objects_list:
        directory_path = "./process/raw_objects/" + object_class + "/"
        pictures_list = os.listdir(directory_path)

        bar = Bar('Processing (2/2) ' + object_class + ':', max=N_PICTURES//2)

        dataset_images = os.listdir("./dataset/images/")
        pic_index = 0

        while pic_index < (N_PICTURES // 2):

            try:
                # Choose an object and random image
                random_index = np.random.randint(0, len(pictures_list))
                random_picture = directory_path + pictures_list[random_index]

                img = cv2.imread(random_picture)
                random_dataset_image = "./dataset/images/" + dataset_images[np.random.randint(0, len(dataset_images))]
                background = cv2.imread(random_dataset_image)

                if background is None:
                    pass

                # Rescale the object
                img_w, img_h = img.shape[1], img.shape[0]
                background_w, background_h = background.shape[1], background.shape[0]

                # Get mask
                filter = get_filter(object_class)
                mask = get_mask(img, np.array(filter[0]), np.array(filter[1]))

                random_ratio = np.random.uniform(low=SCALING_LIMITS[0], high=SCALING_LIMITS[1])
                img = rescale_image(img, random_ratio)
                mask = rescale_image(mask, random_ratio)

                box_x, box_y, box_w, box_h = get_object_box(mask)

                # Crop image
                object = crop_image(img, box_x, box_y, box_w, box_h)
                mask = crop_image(mask, box_x, box_y, box_w, box_h)

                # Combine image
                yolo_class = dataset_manager.get_class_index(object_class)
                combined_image, yolo_x, yolo_y, yolo_w, yolo_h = paste_object(object, mask, background)

                # Save everything
                filename = (random_dataset_image.split("/"))[-1].replace(".jpg", "")
                save_image(filename, combined_image)
                save_yolo_file(filename, yolo_class, yolo_x, yolo_y, yolo_w, yolo_h, mode="a")

                pic_index += 1

                bar.goto(pic_index)

            except:
                pass

    for i in range(4000):
        try:
            background = backgrounds_manager.get_random_background()

            if background is None:
                pass

            filename = str(uuid.uuid4())
            save_image(filename, background)

        except:
            pass

    dataset_manager.save_config()

    print("Dataset generated in %s seconds." % (time.time() - start_time))


if __name__ == "__main__":
    main()
