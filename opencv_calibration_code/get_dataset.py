import cv2
import time
import os
import glob

CALIBRATE_IMGS_FOLDER = "test_folder"

# NOTE: double check these values every time the cameras are plugged back in
LEFT_CAMERA_IDX = 2
RIGHT_CAMERA_IDX = 1


def collect_images():
    output_path = os.path.join(os.getcwd(), CALIBRATE_IMGS_FOLDER)
    print("Running from directory '{}/'; will store calibration images in '{}/'".format(os.getcwd(),
                                                                                        CALIBRATE_IMGS_FOLDER))
    if os.path.exists(output_path):
        if len(os.listdir(output_path)) != 0:
            print(
                "To proceed with collecting the dataset and writing images to '{}/', the contents of the '{}/' directory "
                "need to be deleted. Delete all files in `{}`?".format(output_path, CALIBRATE_IMGS_FOLDER, output_path))
            while True:
                response = input("y/n -> ")
                response = response.lower()
                if response == "y" or response == "yes":
                    files = glob.glob(output_path + "/*")
                    for f in files:
                        print("  deleting '{}'".format(f))
                        os.remove(f)
                    break
                elif response == "n" or response == "no":
                    print("Aborting data collection")
                    exit(-1)
                else:
                    print("Invalid response '{}', please try again".format(response))
                    continue
    else:
        print("Creating directory '{}'".format(output_path))
        os.makedirs(output_path)

    capL = cv2.VideoCapture(LEFT_CAMERA_IDX)
    capR = cv2.VideoCapture(RIGHT_CAMERA_IDX)

    if not capL.isOpened() or not capR.isOpened():
        print("Aborting because one or both of the cameras couldn't be opened")
        exit(-1)

    xml_file = open(os.path.join(output_path, "calibrate_imgs.xml"), 'w')
    xml_text = ['<?xml version="1.0"?><opencv_storage><imagelist>', '', '</imagelist></opencv_storage>']

    # uncomment if 720x1280 resolution is desired
    # capL.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # capL.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # capR.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # capR.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    capL.grab()
    capR.grab()
    _, imgL = capL.retrieve(-1)
    _, imgR = capR.retrieve(-1)
    print(imgL.shape)
    time.sleep(1)
    for i in range(0, 40):
        t = time.time()
        while (True):
            capL.grab()
            capR.grab()
            _, imgL = capL.retrieve(-1)
            _, imgR = capR.retrieve(-1)
            cv2.imshow("left", imgL)
            cv2.imshow("right", imgR)

            if cv2.waitKey(1) & 0xFF == ord('n'):
                break
            if time.time() - t > 1:
                break

        print("grabbing {}".format(i))
        capL.grab()
        capR.grab()

        _, imgL = capL.retrieve()
        _, imgR = capR.retrieve()

        filenameL = "left" + str(i) + ".png"
        filenameR = "right" + str(i) + ".png"

        cv2.imwrite(os.path.join(output_path, filenameL), imgL)
        cv2.imwrite(os.path.join(output_path, filenameR), imgR)
        xml_text[1] += os.path.join('"calibrate_imgs', filenameL) + '" ' + os.path.join('"calibrate_imgs',
                                                                                        filenameR + '" ')
        print("Captured ", i)

    xml_file.write("".join(xml_text))
    xml_file.close()


if __name__ == "__main__":
    collect_images()
