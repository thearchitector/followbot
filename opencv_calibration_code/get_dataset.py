import cv2
import time
import os

def collect_images():
    output_path = os.path.join(os.getcwd(), "test_images")

    if os.path.exists(output_path):
        os.system("rm -rf " + output_path)
    os.makedirs(output_path, exist_ok=False)

    xml_file = open(os.path.join(output_path, "test_images.xml"), 'w')
    xml_text = ['<?xml version="1.0"?><opencv_storage><imagelist>', '', '</imagelist></opencv_storage>']

    capL = cv2.VideoCapture(2)
    capR = cv2.VideoCapture(1)

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
        xml_text[1] += os.path.join('"test_images', filenameL) + '" ' + os.path.join('"test_images', filenameR + '" ')

        print("Captured ", i)

    xml_file.write("".join(xml_text))
    xml_file.close()

if __name__ == "__main__":
    collect_images()