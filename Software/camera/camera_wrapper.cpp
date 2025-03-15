extern "C" {
    #include "camera_wrapper.h"
}

#include <iostream>
//include <libcamera/libcamera.h>
#include <opencv4/opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;

using namespace std;
/*
void captureImage() {
    // C++ logic using libcamera
    libcamera::CameraManager *cameraManager = new libcamera::CameraManager();
    cameraManager->start();

    std::vector<std::shared_ptr<libcamera::Camera>> cameras = cameraManager->cameras();
    if (cameras.empty()) {
        printf("No cameras available\n");
        return;
    }

    std::shared_ptr<libcamera::Camera> camera = cameras[0];

    if (camera->acquire()) {
        printf("Failed to acquire camera\n");
        return;
    }

    std::shared_ptr<libcamera::CameraConfiguration> cameraConfig = camera->generateConfiguration({libcamera::StreamRole::VideoRecording});
    if (!cameraConfig) {
        printf("Failed to generate camera configuration\n");
        return;
    }

    libcamera::StreamConfiguration &config = cameraConfig->at(0);
    config.size = {1920, 1080};  // Set image resolution
    config.pixelFormat = libcamera::formats::YUV420; // Set pixel format

    if (camera->configure(cameraConfig.get())) {
        printf("Failed to configure camera\n");
        return;
    }

    if (camera->start()) {
        printf("Failed to start camera stream\n");
        return;
    }

    std::shared_ptr<libcamera::Request> request = camera->createRequest();
    auto stream = *camera->streams().begin();  // Get the first stream
    request->addBuffer(stream, nullptr);  // Add buffer

    if (camera->queueRequest(request.get())) {
        printf("Failed to queue request\n");
        return;
    }

    //if (camera->waitForRequests()) {
        //printf("Request completed\n");
    //}

    camera->stop();
    camera->release();
    delete cameraManager;
}
*/
void readLED() {
    //get the png photo
    

    cv::Mat image = cv::imread("visual_gray_on.png");

    if (image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return;
    }

    // Convert the image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // Calculate the average intensity of the image
    cv::Scalar meanIntensity = cv::mean(grayImage);

    // The mean intensity is the brightness value
    double brightness = meanIntensity[0];

    std::cout << "Image brightness (average intensity): " << brightness << std::endl;
}

int main() {
    readLED();  // Call the function you want to execute
    return 0;
}

