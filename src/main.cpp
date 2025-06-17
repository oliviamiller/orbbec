#include <iostream> 
#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void saveColor(std::shared_ptr<ob::Frame> colorFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);
    std::string colorName = "color_"+std::to_string(colorFrame->timeStamp() ) + ".jpg";
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
    std::string depthName = "Depth_" + std::to_string(depthFrame->timeStamp()) + ".png";
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

int main() {
    try {
        std::cout << "starting orbbec program" << std::endl;
        ob::Pipeline pipe;

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

        pipe.start(config);

        std::cout << "waiting for frames" << std::endl;

        // Discarding the first few frames since it takes a bit for them to initialize. 
        for (int i = 0; i<5; i++) {
            pipe.waitForFrames();
        }

        auto frames = pipe.waitForFrames();
        if(!frames) {
            std::cerr << "couldnt get frames" << std::endl;
            return -1;
        }

        auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR); // Align depth frame to color frame
        auto alignedFrames = alignFilter->process(frames);
        std::cout << "got frames" <<  std::endl;
        std::cout << frames->getCount() << std::endl;


        auto colorFrame = frames->colorFrame();
        if(!colorFrame) {
            std::cerr << "couldnt get color frame" << std::endl;
        }

        auto depthFrame = frames-> depthFrame();
        if (!depthFrame) {
            std::cerr << "cloudnt get depth frame" << std::endl;
        }

        pipe.stop();

    if (colorFrame) {
            saveColor(colorFrame);
        }
    if (depthFrame) {
            saveDepth(depthFrame);
    }

    makePointCloud(alignedFrames);
}
    catch(ob::Error &e) {
        std::cerr << "error occured" << std::endl;
        }
    return 0;
}

