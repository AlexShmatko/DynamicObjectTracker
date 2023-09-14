#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

const double CENTER = 0.5;
const std::string LEFT = "left";
const std::string RIGHT = "right";
const std::string UP = "up";
const std::string DOWN = "down";

std::pair<std::string, std::string> computeDirection(double x, double y) {
    std::string xDir = "";
    std::string yDir = "";

    if (x < CENTER) xDir = LEFT;
    else if (x > CENTER) xDir = RIGHT;
    if (y < CENTER) yDir = UP;
    else if (y > CENTER) yDir = DOWN;
    return {xDir, yDir};
}

std::pair<double, double> computeAcceleration(const std::string& xDir, const std::string& yDir, double x, double y) {
    double xAccel = 0;
    double yAccel = 0;
    if (xDir == LEFT) xAccel = (CENTER - x);
    else if (xDir == RIGHT) xAccel = (x - CENTER);
    if (yDir == UP) yAccel = (CENTER - y);
    else if (yDir == DOWN) yAccel = (y - CENTER);
    return {xAccel * 100 * 2, yAccel * 100 * 2};
}

void gimbalNavigator(double x, double y, double capWidth, double capHeight) {
    x /= capWidth;
    y /= capHeight;
    auto [xDir, yDir] = computeDirection(x, y);
    auto [xAccel, yAccel] = computeAcceleration(xDir, yDir, x, y);
    std::cout << "X-DIR: " << xDir << ", Y-DIR: " << yDir << ", X-ACCEL: " << xAccel << ", Y-ACCEL: " << yAccel << std::endl;
}

void drawBox(cv::Mat& img, cv::Rect2d bbox) {
    cv::rectangle(img, bbox, cv::Scalar(255, 0, 255), 3, 1);
    cv::putText(img, "Tracking", cv::Point(75, 75), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

bool isTrackingActive = true;
bool isRoiUpdated = false;
bool isTrackerInited = false;
cv::Rect2d roi;
cv::Ptr<cv::TrackerCSRT> tracker = cv::TrackerCSRT::create();

void on_mouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONUP) {
        if (isTrackingActive) {
            roi = cv::Rect2d(x, y, 100, 100);  // width and height are fixed at 100 for now
            isRoiUpdated = true;
        }
    } else if (event == cv::EVENT_RBUTTONUP) {
        roi = cv::Rect2d();
        isRoiUpdated = false;
    }
}

int main() {
    cv::VideoCapture cap(0);  // Camera number in OS

    double capWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double capHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    cv::namedWindow("Tracking");
    cv::setMouseCallback("Tracking", on_mouse);

    while (true) {
        cv::Mat img;
        bool SUCCESS = cap.read(img);

        if (!SUCCESS) {
            std::cerr << "Failed to read from camera!" << std::endl;
            break;
        }

        if (isTrackingActive && isRoiUpdated) {
            tracker->init(img, roi);
            isTrackerInited = true;
            isRoiUpdated = false;
        } else if (isTrackingActive && isTrackerInited) {
            bool success;
            cv::Rect bbox;
            success = tracker->update(img, bbox);

            if (success) {
                drawBox(img, bbox);
                gimbalNavigator(bbox.x, bbox.y, capWidth, capHeight);
            }
        }

        cv::imshow("Tracking", img);

        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    return 0;
}
