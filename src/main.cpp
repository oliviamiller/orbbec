
#include <iostream>
#include <chrono>
#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>

void saveColor(std::shared_ptr<ob::Frame> colorFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);
    std::string colorName = "../Color/color_"+std::to_string(colorFrame->systemTimeStamp() ) + ".jpg";
    cv::Mat colorRawMat(1, colorFrame->dataSize(), CV_8UC1, colorFrame->data());
    cv::Mat colorMat = cv::imdecode(colorRawMat, 1);
    cv::imwrite(colorName, colorMat, compression_params);
    std::cout << "color saved" << std::endl;
}


void saveDepth(std::shared_ptr<ob::Frame> depthFrame) {
    std::vector<int> compression_params;
   // compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    //compression_params.push_back(90);
    // System time stamp should be the UNIX time in ms
    std::string depthName = "../Depth/Depth_" + std::to_string(depthFrame->systemTimeStamp()) + ".png";
    auto videoFrame = std::dynamic_pointer_cast<ob::VideoFrame>(depthFrame);
    cv::Mat depthMat(videoFrame->height(), videoFrame->width(), CV_16UC1, depthFrame->data());
    cv::imwrite(depthName, depthMat, compression_params);
    std::cout << "Depth saved" << std::endl;
}

void saveRGBPointsToPCD(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    start = std::chrono::high_resolution_clock::now();

    int   pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");

    if(!fp) {
        throw std::runtime_error("Failed to open file for writing");
    }

    OBColorPoint    *points            = (OBColorPoint *)frame->data();
    int               validPointsCount = 0;
    static const auto min_distance     = 1e-6;

    // First pass: Count valid points (non-zero points)
    for(int i = 0; i < pointsSize; i++) {
        if(fabs(points[i].x) >= min_distance || fabs(points[i].y) >= min_distance || fabs(points[i].z) >= min_distance) {
            validPointsCount++;
        }
    }

    fprintf(fp, "VERSION .7\n");
    fprintf(fp, "FIELDS x y z rgb\n");
    fprintf(fp, "SIZE 4 4 4 4\n");
    fprintf(fp, "TYPE F F F F\n");
    fprintf(fp, "COUNT 1 1 1 1\n");
    fprintf(fp, "WIDTH %d\n", validPointsCount);  // Use valid points count
    fprintf(fp, "HEIGHT 1\n");
    fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(fp, "POINTS %d\n", validPointsCount);
    fprintf(fp, "DATA binary\n");

    struct PointXYZRGB {
        float x, y, z;
        float rgb;
    };

    std::vector<PointXYZRGB> pcdPoints;
    pcdPoints.reserve(validPointsCount);



    for (int i = 0; i < pointsSize; i++) {
        OBColorPoint& p = points[i];
        if(fabs(p.x) >= min_distance || fabs(p.y) >= min_distance || fabs(p.z) >= min_distance) {
            unsigned int r = (unsigned int)p.r;
            unsigned int g = (unsigned int)p.g;
            unsigned int b = (unsigned int)p.b;
            unsigned int rgb_int = (r << 16) | (g << 8) |b;
            float rgb_float;
            memcpy(&rgb_float, &rgb_int, sizeof(float));
            PointXYZRGB pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            std::memcpy(&pt.rgb, &rgb_float, sizeof(float));
            pcdPoints.push_back(pt);
        }
    }

    fwrite(pcdPoints.data(), sizeof(PointXYZRGB), validPointsCount, fp);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
    std::cout << "duration: " << duration.count() << "ms" << std::endl;

    fflush(fp);
    fclose(fp);


}

void makePointCloud(std::shared_ptr<ob::Frame> frames) {
    auto pointcloud = std::make_shared<ob::PointCloudFilter>();
    pointcloud->setCreatePointFormat(OB_FORMAT_RGB_POINT);
    std::shared_ptr<ob::Frame> frame = pointcloud->process(frames);
    std::string pcdName = "pcd_" + std::to_string(frame->timeStamp()) + ".pcd";
    saveRGBPointsToPCD(frame, pcdName);
}


void frameCallback(std::shared_ptr<ob::FrameSet> frameset) {
    // if (!frameset) {
    //     std::cerr << "no frames" << std::endl;
    //     return;
    // }

    // auto color = frameset->colorFrame();
    // auto depth = frameset->depthFrame();


    // auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR); // Align depth frame to color frame
    // auto alignedFrames = alignFilter->process(frameset);
    // std::cout << "got frames" <<  std::endl;
    // std::cout << frameset->getCount() << std::endl;


    // auto colorFrame = frameset->colorFrame();
    // if(!colorFrame) {
    //     std::cerr << "couldnt get color frame" << std::endl;
    // }

    // auto depthFrame = frameset-> depthFrame();
    // if (!depthFrame) {
    //     std::cerr << "couldnt get depth frame" << std::endl;
    // }


    //  // Run this in the background so we dont miss frames
    // std::thread([=]() {
    //     if (colorFrame) saveColor(colorFrame);
    //     if (depthFrame) saveDepth(depthFrame);
    // }).detach();
}

void deviceChangedCallback(std::shared_ptr<DeviceList> added, std::shared_ptr<DeviceList> removed) {
    // device(s) have been connecxted
    if (added && added->deviceCount() > 0) {
        std::cout << "new device has been connected:\n";
        for (uint32_t i = 0; i < added->deviceCount(); ++i) {
            auto devInfo = added->getDevice(i)->getDeviceInfo();
            std::cout << " Serial number: " << devInfo->getSerialNumber() << std::endl;
        }
    }

    if (removed && removed->deviceCount() > 0) {
        std::cout << "Device(s) disconnected:\n";
        for (uint32_t i = 0; i < removed->deviceCount(); ++i) {
            auto devInfo = removed->getDevice(i)->getDeviceInfo();
            std::cout << "  - Serial: " << devInfo->getSerialNumber() << std::endl;
        }
    }
}

int main() {
    std::cout << "starting orbbec program" << std::endl;
    ob::Pipeline pipe;


    ob::Context context;
    auto deviceList = context.queryDeviceList();
    int deviceCount = deviceList->deviceCount();
    std::cout << "Found " << deviceCount << " devices." << std::endl;
    for (int i = 0; i < deviceCount; ++i) {
        auto device = deviceList->getDevice(i);
        auto info = device->getDeviceInfo();

        std::string serial = info->getSerialNumber();
        std::string name = info->getName();
        std::string uid = info->getUid();

        std::cout << "Device " << i+1 << ":\n"
        << "  Name:   " << name << "\n"
        << "  Serial Number: " << serial << "\n"
        << "  UID:    " << uid << "\n";
    }

    context.setDeviceChangedCallback(deviceChangedCallback);

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    auto depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
    if(depthProfiles) {
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
    }

    std :: cout << "enabling depth stream" << std::endl;
    config->enableStream(depthProfile);

    auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    if (colorProfiles) {
        colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
    }
    std :: cout << "enabling color stream" << std::endl;
    config -> enableStream(colorProfile);

    // ensure depth and color are synchronized.
    pipe.enableFrameSync();

    //pipe.start(config);
    pipe.start(config, frameCallback);


    auto intr = depthProfile -> getIntrinsic();
    std::cout << "depth instrinsics: " << std::endl;
    std::cout << "  Width: " << intr.width << "\n";
    std::cout << "  Height: " << intr.height << "\n";
    std::cout << "  Fx: " << intr.fx << "\n";
    std::cout << "  Fy: " << intr.fy << "\n";
    std::cout << "  Cx: " << intr.cx << "\n";
    std::cout << "  Cy: " << intr.cy << "\n";
    std::cout << "\n\n";


    auto colorIntr = colorProfile -> getIntrinsic();
    std::cout << "color instrinsics: " << std::endl;
    std::cout << "  Width: " << colorIntr.width << "\n";
    std::cout << "  Height: " << colorIntr.height << "\n";
    std::cout << "  Fx: " << colorIntr.fx << "\n";
    std::cout << "  Fy: " << colorIntr.fy << "\n";
    std::cout << "  Cx: " << colorIntr.cx << "\n";
    std::cout << "  Cy: " << colorIntr.cy << "\n";
    std::cout << "\n\n";


     while (true) {
        //std::cout << "waiting for frames" << std::endl;
       // std::this_thread::sleep_for(std::chrono::seconds(1));
     }

    // // // Discarding the first few frames since it takes a bit for them to initialize.
    // for (int i = 0; i<5; i++) {
    //     pipe.waitForFrames();
    // }

    // auto frames = pipe.waitForFrames();
    // if(!frames) {
    //     std::cerr << "couldnt get frames" << std::endl;
    //     return -1;
    // }

    // auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR); // Align depth frame to color frame
    // auto alignedFrames = alignFilter->process(frames);
    // std::cout << "got frames" <<  std::endl;
    // std::cout << frames->getCount() << std::endl;


    // auto colorFrame = frames->colorFrame();
    // if(!colorFrame) {
    //     std::cerr << "couldnt get color frame" << std::endl;
    // }

    // auto depthFrame = frames-> depthFrame();
    // if (!depthFrame) {
    //     std::cerr << "cloudnt get depth frame" << std::endl;
    // }

    //pipe.stop();

// if (colorFrame) {
//         saveColor(colorFrame);
//     }
// if (depthFrame) {
//         saveDepth(depthFrame);
// }

// makePointCloud(alignedFrames);

    return 0;
}

