/* person_detect.cpp
 * FollowBot POE Project
 * @author: Duncan Mazza & Elias Gabriel
 *
 * Contains definitions for the HumanDetector class.
 *
 * Note:
 * A subset of this file's code has been adapted from other sources, including a tutorial written by BigVision LLC and
 * an example directly from OpenCV's repository (that the other tutorial was based on itself):
 * https://github.com/opencv/opencv/blob/master/samples/dnn/object_detection.cpp
 *
 * OpenCV license: http://opencv.org/license.html
 *
 */

#include <human.hpp>

void HumanDetector::setupNetwork() {
    /*
     * Instantiates the network object
     */
    net = cv::dnn::readNetFromDarknet(MODEL_CONFIG, MODEL_WEIGHTS);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

std::vector<cv::String> HumanDetector::getOutputsNames() {
    /*
     * Get the names of the output layers
     * TODO: Improve documentation
     */
    static std::vector<cv::String> names;

    if (names.empty()) {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i) names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

bool HumanDetector::postProcess(cv::Mat &frame, const std::vector<cv::Mat> &outs, cv::Rect &detected) {
    /*
     * TODO: documentation
     */
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (const auto &out : outs) {
        // Loop through all the bounding boxes output from the network and keep only the ones with high confidence
        // scores and boxes that are identified to be of a person; assign the box's class label as the class with
        // the highest score for the box.
        auto *data = (float *) out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols) {
            cv::Mat scores = out.row(j).colRange(5, out.cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, nullptr, &confidence, nullptr, &classIdPoint);
            if (confidence > CONF_THRESHOLD && classIdPoint.x == 0) {  // filter to only detect people
                int width = (int) (data[2] * (float) frame.cols);
                int height = (int) (data[3] * (float) frame.rows);
                int left = (int) (data[0] * (float) frame.cols) - width / 2;
                int top = (int) (data[1] * (float) frame.rows) - height / 2;
                confidences.push_back((float) confidence);
                boxes.emplace_back(left, top, width, height);
            }
        }
    }

    std::vector<int> indices;
    if (!boxes.empty()) {
        // Perform non maximum suppression to eliminate redundant overlapping boxes with lower confidences
        cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, XMAX_SUPPRESSION_THRESHOLD, indices);

        // Find and keep only the box with the highest confidence
        double maxConfidence = 0;
        int maxIdx = 0;
        for (int idx : indices) {
            if (confidences[idx] > maxConfidence) {
                maxIdx = idx;
                maxConfidence = confidences[idx];
            }
        }

        // Shrink the detected box to maximize the probability that it encompasses exclusively pixels of a person
        detected = boxes[maxIdx];
        int oldWidth = detected.width;
        int oldHeight = detected.height;
        detected.width *= BOX_X_SCALE;
        detected.height *= BOX_Y_SCALE;
        detected.x += (int) (0.5 * (float) (oldWidth - detected.width));
        detected.y += (int) (0.5 * (float) (oldHeight - detected.height));

        #ifndef PRODUCTION
        drawPred(confidences[maxIdx], detected.x, detected.y, detected.x + detected.width, detected.y + detected.height, frame);
        imshow("Person Detection", frame);
        cv::waitKey(1);
        #endif
        return true;
    }
    return false;
}

bool HumanDetector::detect(cv::Mat &frame, cv::Rect &detected) {
    cv::Mat blob;
    // Create a 4D blob from a frame.
    cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(FRAME_WIDTH, FRAME_HEIGHT), cv::Scalar(0, 0, 0), true, false);
    // Sets the input to the network
    net.setInput(blob);
    // Runs the forward pass to get output of the output layers
    std::vector<cv::Mat> outs;
    net.forward(outs, getOutputsNames());
    // Remove the bounding boxes with low confidence
    return postProcess(frame, outs, detected);
}

void HumanDetector::getHumanPosition(cv::Mat &rectifiedImg, cv::Mat &pointcloud, followbot::World &world_msg) {
    /*
     * Finds the location of a person in a point cloud given a rectified image from one of the cameras and a matrix of
     * the reprojected point cloud. Sets this location in the world_msg parameter, and sets the location to (0, 0) if
     * a person is not found.
     */
    cv::Rect detected;
    float xSum = 0;
    float zSum = 0;
    float count = 1.0;

    if (detect(rectifiedImg, detected)) {
        cv::Point3f xyz_;
        // Loop through the image matrix containing the reprojected points and find the centroid of the points
        // identified to belong to the person
        for (int row = detected.y; row <= detected.y + detected.height; row++) {
            for (int col = detected.x; col <= detected.x + detected.width; col++) {
                xyz_ = pointcloud.at<cv::Point3f>(row, col);
                if (xyz_.z <= DIST_LIMIT) {
                    xSum += xyz_.x;
                    zSum += xyz_.z;
                    count++;
                }
            }
        }
    }
    world_msg.person.x = xSum / count;
    world_msg.person.z = zSum / count;
}

void HumanDetector::drawPred(float conf, int left, int top, int right, int bottom, cv::Mat &frame) {
    /*
     * Draw a bounding box around the identified person
     */
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    std::string label = cv::format("%.2f", conf);  // class name and its confidence
    label = "person";

    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = cv::max(top, labelSize.height);
    rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)),
              cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}
