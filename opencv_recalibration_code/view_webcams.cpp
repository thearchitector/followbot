#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    VideoCapture stream1(2);   //0 is the id of video device.0 if you have only one camera.
    VideoCapture stream2(1);

    if (!stream1.isOpened() || !stream2.isOpened()) { //check if video device has been initialised
        cout << "cannot open one or both cameras" << endl;
        exit(-1);
    }

//unconditional loop
    while (true) {
        Mat cameraFrame1;
        Mat cameraFrame2;
        stream1.grab();
        stream2.grab();
        stream1.retrieve(cameraFrame1);
        stream2.retrieve(cameraFrame2);
        imshow("cam1", cameraFrame1);
        imshow("cam2", cameraFrame2);
        if ((char)waitKey(500) == 'q')
            break;

    }
    return 0;
}