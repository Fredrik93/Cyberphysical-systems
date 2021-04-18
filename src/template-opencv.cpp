/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env) {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            //Values for HSV blue cones
            int minHB = 51;
            int maxHB = 145;
            int minSB = 93;
            int maxSB = 199;
            int minVB = 42;
            int maxVB = 215;

            //Values for HSV yellow cones
            int minHY = 0;
            int maxHY = 42;
            int minSY = 75;
            int maxSY = 212;
            int minVY = 170;
            int maxVY = 240;

            //Vectors for the shape detection
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;

            //Variables for dilation
            int dilationSize = 0;

            //Variables used for the framecounter
            int startTime = 0;
            int endTime = 0;
            int framecounter = 0;

            //Variables used for detection algorithm
            int frameLimiter = 0;
            int frameMax = 4;
            int yellowRight = 0; //if yellow on right side = 1, if yellow not on right side = 0
            int groundSteeringWheel = 0;

            //Variables used for steering wheel angle computation
            int turningBlue = 1;
            int turningYellow = 1;
            int yellowSet = 0;

            //blobb detection varaible
            int blobSize = 60;

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
                int coneFound = 0;

                //Framecounter code
                framecounter++;
                cluon::data::TimeStamp start{cluon::time::now()};
                time_t rawtimeStart = start.seconds();
                struct tm ts2;
                char buffer2[80];

                ts2 = *localtime(&rawtimeStart);
                strftime(buffer2, sizeof(buffer2), "%S", &ts2);
                startTime = atoi(buffer2);

                // OpenCV data structure to hold an image.
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }

                sharedMemory->unlock();

                //Side detection algorithm
                if (frameLimiter < frameMax)
                {
                    //Select roiDetect
                    cv::Rect detectRegionOfInterest = cv::Rect(395, 250, 230, 130);
                    cv::Mat roiDetect = img(detectRegionOfInterest);

                    //HSV operation yellow
                    cv::Mat imgHSVyDetect;
                    cvtColor(roiDetect, imgHSVyDetect, cv::COLOR_BGR2HSV);
                    cv::Mat imgColorSpaceYellowDetect;
                    cv::inRange(imgHSVyDetect, cv::Scalar(minHY, minSY, minVY), cv::Scalar(maxHY, maxSY, maxVY), imgColorSpaceYellowDetect);

                    //Apply gaussian blur
                    cv::Mat blurPicDetection;
                    cv::GaussianBlur(imgColorSpaceYellowDetect, blurPicDetection, cv::Size(9, 9), 1.0);

                    //Blob detection
                    cv::findContours(blurPicDetection, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
                    cv::Mat imgWithContoursYellow = cv::Mat::zeros(blurPicDetection.rows, blurPicDetection.cols, CV_8UC3);
                    cv::RNG rng3(12345);
                    for (unsigned int i = 0; i < contours.size(); i++)
                    {
                        //look for blobs of color bigger 60 pixels
                        if (cv::contourArea(contours[i]) > blobSize)
                        {
                            cv::Scalar colorYellow = cv::Scalar(rng3.uniform(50, 255), rng3.uniform(50, 255), rng3.uniform(50, 255));
                            cv::drawContours(imgWithContoursYellow, contours, i, colorYellow, 1, 8, hierarchy, 0);
                            //check flag to see if value has been set yet
                            if (yellowRight != 1)
                            {
                                yellowRight = 1;
                                //adjust direction for blue cone detection accordingly
                                turningBlue = turningBlue * -1;
                            }
                        }
                    }
                    frameLimiter++;
                }

                //If no yellow cones have been found in the prior operation by checking
                if (frameLimiter == frameMax && yellowRight == 0 && yellowSet != 1)
                {
                    turningYellow = turningYellow * -1;
                    yellowSet = 1;
                }

                //Detection algorithm
                if (frameLimiter == frameMax)
                {
                    cv::rectangle(img, cv::Point(395, 250), cv::Point(625, 380), cv::Scalar(0, 150, 150));

                    //Select Main ROI
                    cv::Rect mainRegionOfInterest = cv::Rect(260, 275, 135, 85); //position (x,y), size (width, height)
                    cv::Mat roiMain = img(mainRegionOfInterest);

                    //HSV operation blue
                    cv::Mat imgHSVb;
                    cvtColor(roiMain, imgHSVb, cv::COLOR_BGR2HSV);
                    cv::Mat imgColorSpaceBlue;
                    cv::inRange(imgHSVb, cv::Scalar(minHB, minSB, minVB), cv::Scalar(maxHB, maxSB, maxVB), imgColorSpaceBlue);

                    //Apply gaussian blur
                    cv::Mat blueBlur;
                    cv::GaussianBlur(imgColorSpaceBlue, blueBlur, cv::Size(9, 9), 4.0);

                    //Apply dilation
                    int dilationType = cv::MORPH_ELLIPSE;
                    cv::Mat element;
                    cv::Mat dilationDst;
                    element = getStructuringElement(dilationType,
                                                    cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                                    cv::Point(dilationSize, dilationSize));
                    cv::dilate(blueBlur, dilationDst, element);

                    //Blob detection
                    cv::findContours(dilationDst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
                    cv::Mat imgWithContours = cv::Mat::zeros(dilationDst.rows, dilationDst.cols, CV_8UC3);
                    cv::RNG rng(12345);
                    for (unsigned int i = 0; i < contours.size(); i++)
                    {
                        //look for blobs of color bigger 60 pixels
                        if (cv::contourArea(contours[i]) > blobSize)
                        {
                            cv::Scalar color = cv::Scalar(rng.uniform(50, 255), rng.uniform(50, 255), rng.uniform(50, 255));
                            cv::drawContours(imgWithContours, contours, i, color, 1, 8, hierarchy, 0);
                            //set flag that cone has been found
                            coneFound = 1;
                            //if groundsteering has not reached maximum value
                            if (groundSteeringWheel > -12 && groundSteeringWheel < 12)
                            {
                                groundSteeringWheel += turningBlue;
                            }
                            break;
                        }
                    }
                    if (VERBOSE)
                    {
                        cv::imshow("Blue", imgWithContours);
                        cv::waitKey(1);
                    }

                    cv::Mat imgWithContoursYellow;
                    //if no cone has been found by now
                    if (coneFound != 1)
                    {
                        //HSV operation yellow
                        cv::Mat imgHSVy;
                        cvtColor(roiMain, imgHSVy, cv::COLOR_BGR2HSV);
                        cv::Mat imgColorSpaceYellow;
                        cv::inRange(imgHSVy, cv::Scalar(minHY, minSY, minVY), cv::Scalar(maxHY, maxSY, maxVY), imgColorSpaceYellow);

                        //Apply gaussian blur
                        cv::Mat blurPic2;
                        cv::GaussianBlur(imgColorSpaceYellow, blurPic2, cv::Size(9, 9), 1.0);

                        //Blob detection
                        cv::findContours(blurPic2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
                        imgWithContoursYellow = cv::Mat::zeros(blurPic2.rows, blurPic2.cols, CV_8UC3);
                        cv::RNG rng2(12345);
                        for (unsigned int i = 0; i < contours.size(); i++)
                        {
                            if (cv::contourArea(contours[i]) > blobSize)
                            {
                                cv::Scalar colorYellow = cv::Scalar(rng2.uniform(50, 255), rng2.uniform(50, 255), rng2.uniform(50, 255));
                                cv::drawContours(imgWithContoursYellow, contours, i, colorYellow, 1, 8, hierarchy, 0);
                                if (VERBOSE)
                                {
                                    cv::imshow("Yellow", imgWithContoursYellow);
                                    cv::waitKey(1);
                                }
                                //set flag that cone has been found
                                coneFound = 1;
                                //if groundsteering has not reached maximum value
                                if (groundSteeringWheel > -12 && groundSteeringWheel < 12)
                                {
                                    groundSteeringWheel += turningYellow;
                                }
                                break;
                            }
                        }
                    }
                   
                    //If no cone has been detected in the current frame
                    if (coneFound == 0)
                    {
                        if (groundSteeringWheel > 0 || groundSteeringWheel < 0)
                        {
                            groundSteeringWheel = 0;
                        }
                    }

                    //Timestamp code
                    cluon::data::TimeStamp end{cluon::time::now()};
                    time_t rawtimeEnd = end.seconds();
                    struct tm ts3;
                    char buffer3[80];

                    //Framecounter
                    ts3 = *localtime(&rawtimeEnd);
                    strftime(buffer3, sizeof(buffer3), "%S", &ts3);
                    endTime = atoi(buffer3);
                    if (endTime > startTime || endTime == 59)
                    {
                        //std::cout << "FPS: " << framecounter << std::endl;
                        std::string fps = std::to_string(framecounter);
                        cv::putText(img, //target image
                                    fps,
                                    cv::Point(610, 25), //top-left position
                                    cv::FONT_HERSHEY_PLAIN,
                                    1.0,
                                    CV_RGB(255, 255, 255), //font color
                                    2);
                        framecounter = 0;
                    }

                    //Draw a red rectangle on the detectROI and display image
                    cv::rectangle(img, cv::Point(260, 290), cv::Point(395, 365), cv::Scalar(0, 0, 255));
                    cv::putText(img, //target image
                                "Group_07",
                                cv::Point(15, 25), //top-left position
                                cv::FONT_HERSHEY_PLAIN,
                                1.0,
                                CV_RGB(255, 255, 255), //font color
                                2);
                }

                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                }

                // Display image on your screen
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }

                //Get timestamp and print steering wheel angle
                sharedMemory->lock();
                {
                    //https://chrberger.github.io/libcluon/classcluon_1_1SharedMemory.html#a37eec40b45c7c4301ffa602772178661
                    std::pair<bool, cluon::data::TimeStamp> stampCurrentPair = sharedMemory->getTimeStamp();
                    cluon::data::TimeStamp stampCurrent = stampCurrentPair.second;
                    std::cout << "group_07;" << stampCurrent.seconds() << stampCurrent.microseconds() << ";" << (double)groundSteeringWheel / 40 << std::endl;
                }
                sharedMemory->unlock();
            }
        }
        retCode = 0;
    }
    return retCode;
}
