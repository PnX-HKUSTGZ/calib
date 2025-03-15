// 在头文件部分添加多线程支持库
#include <iostream>
#include <string>
#include <ctime>
#include <cstdio>
#include <thread>             // 添加线程支持
#include <mutex>              // 添加互斥量支持
#include <condition_variable> // 添加条件变量支持
#include <queue>              // 添加队列支持
#include <atomic>             // 添加原子操作支持
#include <functional>         // 添加函数对象支持
#include <future>             // 添加future支持

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "Camera.h"
using namespace cv;
using namespace std;
using namespace hnurm;

// 添加一个简单的线程池实现
class ThreadPool {
public:
    ThreadPool(size_t threads) : stop(false) {
        for(size_t i = 0; i < threads; ++i)
            workers.emplace_back(
                [this] {
                    for(;;) {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lock(this->queue_mutex);
                            this->condition.wait(lock,
                                [this]{ return this->stop || !this->tasks.empty(); });
                            if(this->stop && this->tasks.empty())
                                return;
                            task = std::move(this->tasks.front());
                            this->tasks.pop();
                        }
                        task();
                    }
                }
            );
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared< std::packaged_task<return_type()> >(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...)
            );
            
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            // 不要在停止后添加任务
            if(stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");

            tasks.emplace([task](){ (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for(std::thread &worker: workers)
            worker.join();
    }
    
private:
    // 需要保持追踪线程，以便我们可以join它们
    std::vector<std::thread> workers;
    // 任务队列
    std::queue<std::function<void()>> tasks;
    
    // 同步
    std::mutex queue_mutex;
    std::condition_variable condition;
    // 线程池停止标志
    bool stop;
};

// 线程安全的图像帧队列
class SafeImageQueue {
public:
    SafeImageQueue() : done(false) {}
    
    void push(const Mat& image) {
        std::unique_lock<std::mutex> lock(mtx);
        frames.push(image.clone());
        lock.unlock();
        cond.notify_one();
    }
    
    bool pop(Mat& image) {
        std::unique_lock<std::mutex> lock(mtx);
        cond.wait(lock, [this]{ return !frames.empty() || done; });
        
        if(frames.empty() && done)
            return false;
            
        image = frames.front();
        frames.pop();
        return true;
    }
    
    void setDone() {
        std::unique_lock<std::mutex> lock(mtx);
        done = true;
        lock.unlock();
        cond.notify_all();
    }
    
    bool isEmpty() {
        std::unique_lock<std::mutex> lock(mtx);
        return frames.empty();
    }
    
private:
    std::queue<Mat> frames;
    std::mutex mtx;
    std::condition_variable cond;
    bool done;
};

// 线程安全的特征点结果队列
class SafePointsQueue {
public:
    void push(const vector<Point2f>& points) {
        std::unique_lock<std::mutex> lock(mtx);
        pointsQueue.push_back(points);
    }
    
    vector<vector<Point2f>> getAll() {
        std::unique_lock<std::mutex> lock(mtx);
        vector<vector<Point2f>> result = pointsQueue;
        return result;
    }
    
    size_t size() {
        std::unique_lock<std::mutex> lock(mtx);
        return pointsQueue.size();
    }
    
    // 添加清空方法
    void clear() {
        std::unique_lock<std::mutex> lock(mtx);
        pointsQueue.clear();
    }
    
private:
    vector<vector<Point2f>> pointsQueue;
    std::mutex mtx;
};

class Settings
{
public:
    Settings()
    {
        goodInput = false;
        threadCount = std::thread::hardware_concurrency(); // 默认使用所有可用CPU核心
        if (threadCount == 0) threadCount = 6; // 如果无法确定硬件并发性，使用6个线程
    }
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };
    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
           << "BoardSize_Width" << boardSize.width
           << "BoardSize_Height" << boardSize.height
           << "Square_Size" << squareSize
           << "Calibrate_Pattern" << patternToUse
           << "Calibrate_NrOfFrameToUse" << nrFrames
           << "Calibrate_FixAspectRatio" << aspectRatio
           << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
           << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

           << "Write_DetectedFeaturePoints" << writePoints
           << "Write_extrinsicParameters" << writeExtrinsics
           << "Write_gridPoints" << writeGrid
           << "Write_outputFileName" << outputFileName

           << "Show_UndistortedImage" << showUndistorsed

           << "Input_FlipAroundHorizontalAxis" << flipVertical
           << "Input_Delay" << delay
           << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
        node["Write_gridPoints"] >> writeGrid;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        node["Fix_K1"] >> fixK1;
        node["Fix_K2"] >> fixK2;
        node["Fix_K3"] >> fixK3;
        node["Fix_K4"] >> fixK4;
        node["Fix_K5"] >> fixK5;

        validate();
    }
    void validate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
            inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (isListOfImages(input) && readStringList(input, imageList))
                {
                    inputType = IMAGE_LIST;
                    nrFrames = (nrFrames < (int) imageList.size()) ? nrFrames : (int) imageList.size();
                }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
            {
                inputCapture.OpenCam();
            }
            if (inputType == VIDEO_FILE)
            {
                inputCapture.OpenCam();
            }
            if (inputType != IMAGE_LIST && !true)
                inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Input does not exist: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
        if(fixK1)                  flag |= CALIB_FIX_K1;
        if(fixK2)                  flag |= CALIB_FIX_K2;
        if(fixK3)                  flag |= CALIB_FIX_K3;
        if(fixK4)                  flag |= CALIB_FIX_K4;
        if(fixK5)                  flag |= CALIB_FIX_K5;

        if (useFisheye) {
            // the fisheye model has its own enum, so overwrite the flags
            flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
            if(fixK1)                   flag |= fisheye::CALIB_FIX_K1;
            if(fixK2)                   flag |= fisheye::CALIB_FIX_K2;
            if(fixK3)                   flag |= fisheye::CALIB_FIX_K3;
            if(fixK4)                   flag |= fisheye::CALIB_FIX_K4;
            if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
        }

        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;
        if (true)
        {
            cv::Mat ifo;
            inputCapture.SendFrame(ifo);
            Mat view0 = ifo;
            view0.copyTo(result);
        }
        else if (atImageList < imageList.size())
            result = imread(imageList[atImageList++], IMREAD_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }

    static bool isListOfImages( const string& filename)
    {
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool writeGrid;              // Write refined 3D target grid points
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration
    bool fixK1;                  // fix K1 distortion coefficient
    bool fixK2;                  // fix K2 distortion coefficient
    bool fixK3;                  // fix K3 distortion coefficient
    bool fixK4;                  // fix K4 distortion coefficient
    bool fixK5;                  // fix K5 distortion coefficient
    unsigned int threadCount;

    int cameraID;
    vector<string> imageList;
    size_t atImageList;
    HKcam inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object);

                           bool findChessboardCornersTask(const Mat& view, const Size& boardSize, vector<Point2f>& pointBuf, 
                            int chessBoardFlags, Settings::Pattern calibrationPattern, int winSize) {
  bool found = false;
  
  switch(calibrationPattern) {
      case Settings::CHESSBOARD:
          found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
          if (found) {
              Mat viewGray;
              cvtColor(view, viewGray, COLOR_BGR2GRAY);
              cornerSubPix(viewGray, pointBuf, Size(winSize, winSize),
                          Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
          }
          break;
      case Settings::CIRCLES_GRID:
          found = findCirclesGrid(view, boardSize, pointBuf);
          break;
      case Settings::ASYMMETRIC_CIRCLES_GRID:
          found = findCirclesGrid(view, boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
          break;
      default:
          found = false;
          break;
  }
  
  return found;
}

int main(int argc, char* argv[])
{
    const String keys
        = "{help h usage ? |           | print this message            }"
          "{@settings      |default.xml| input setting file            }"
          "{d              |           | actual distance between top-left and top-right corners of "
          "the calibration grid }"
          "{winSize        | 11        | Half of search window for cornerSubPix }";
    CommandLineParser parser(argc, argv, keys);
    parser.about("This is a camera calibration sample.\n"
                 "Usage: camera_calibration [configuration_file -- default ./default.xml]\n"
                 "Near the sample file you'll find the configuration file, which has detailed help of "
                 "how to edit it. It may be any OpenCV supported file format XML/YAML.");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    //! [file_read]
    Settings s;
    const string inputSettingsFile = parser.get<string>(0);
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    float grid_width = s.squareSize * (s.boardSize.width - 1);
    bool release_object = false;
    if (parser.has("d")) {
        grid_width = parser.get<float>("d");
        release_object = true;
    }

    // 创建线程池
    ThreadPool pool(s.threadCount);

    // 创建安全队列用于存储检测到的特征点
    SafePointsQueue pointsQueue;
    
    // 创建用于显示的结果
    Mat displayView;
    std::mutex displayMutex;
    bool hasNewDisplay = false;
    
    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    // 记录正在处理的任务数
    std::atomic<int> activeTasks(0);

    // 获取窗口大小参数
    int winSize = parser.get<int>("winSize");
    
    //! [get_input]
    for(;;)
    {
        Mat view;
        bool blinkOutput = false;

        view = s.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if(mode == CAPTURING && pointsQueue.size() >= (size_t)s.nrFrames)
        {
            // 等待所有任务完成
            while(activeTasks > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            // 获取所有结果
            imagePoints = pointsQueue.getAll();
            
            if(runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, grid_width, release_object))
                mode = CALIBRATED;
            else
                mode = DETECTION;
        }
        
        if(view.empty())          // If there are no more images stop the loop
        {
            // 等待所有任务完成
            while(activeTasks > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            // 获取所有结果
            if(mode != CALIBRATED && !pointsQueue.getAll().empty()) {
                imagePoints = pointsQueue.getAll();
                runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, grid_width, release_object);
            }
            break;
        }
        
        imageSize = view.size();  // Format input image.
        if(s.flipVertical)
            flip(view, view, 0);

        // 为每一帧创建一个工作任务
        if(mode == CAPTURING || mode == DETECTION) {
            Mat frameCopy = view.clone(); // 创建副本以确保线程安全
            activeTasks++;
            
            // 提交任务到线程池
            pool.enqueue([&, frameCopy]() {
                vector<Point2f> pointBuf;
                int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
                
                if(!s.useFisheye) {
                    chessBoardFlags |= CALIB_CB_FAST_CHECK;
                }
                
                bool found = findChessboardCornersTask(frameCopy, s.boardSize, pointBuf, 
                                                     chessBoardFlags, s.calibrationPattern, winSize);
                
                if(found) {
                    std::unique_lock<std::mutex> displayLock(displayMutex);
                    
                    if(mode == CAPTURING) {
                        pointsQueue.push(pointBuf);
                        blinkOutput = true;
                    }
                    
                    // 在图像上绘制棋盘格角点
                    Mat viewCopy = frameCopy.clone();
                    drawChessboardCorners(viewCopy, s.boardSize, Mat(pointBuf), found);
                    
                    string msg;
                    if(mode == CAPTURING)
                        msg = format("%d/%d", (int)pointsQueue.size(), s.nrFrames);
                    else
                        msg = "Press 'g' to start";
                    
                    int baseLine = 0;
                    Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
                    Point textOrigin(viewCopy.cols - 2*textSize.width - 10, viewCopy.rows - 2*baseLine - 10);
                    
                    putText(viewCopy, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);
                    
                    if(blinkOutput)
                        bitwise_not(viewCopy, viewCopy);
                    
                    displayView = viewCopy;
                    hasNewDisplay = true;
                }
                
                activeTasks--;
            });
        }
        
        // 处理校准后的图像
        if(mode == CALIBRATED) {
            if(s.showUndistorsed) {
                Mat temp = view.clone();
                if(s.useFisheye)
                    cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
                else
                    undistort(temp, view, cameraMatrix, distCoeffs);
            }
            
            string msg = "Calibrated";
            int baseLine = 0;
            Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
            Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
            putText(view, msg, textOrigin, 1, 1, GREEN);
            
            {
                std::unique_lock<std::mutex> displayLock(displayMutex);
                displayView = view.clone();
                hasNewDisplay = true;
            }
        }
        
        // 显示图像
        {
            std::unique_lock<std::mutex> displayLock(displayMutex);
            if(hasNewDisplay) {
                namedWindow("Image View", WINDOW_NORMAL);
                imshow("Image View", displayView);
                hasNewDisplay = false;
            } else if(mode == DETECTION) {
                // 如果没有检测到角点，显示原始图像
                namedWindow("Image View", WINDOW_NORMAL);
                
                string msg = "Press 'g' to start";
                int baseLine = 0;
                Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
                Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
                putText(view, msg, textOrigin, 1, 1, RED);
                
                imshow("Image View", view);
            }
        }
        
        char key = (char)waitKey(50);
        
        if(key == ESC_KEY)
            break;
            
        if(key == 'u' && mode == CALIBRATED)
            s.showUndistorsed = !s.showUndistorsed;
            
        if(key == 'g' && mode != CALIBRATED) {
            mode = CAPTURING;
            pointsQueue.clear(); // 使用新添加的 clear 方法清空队列
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;

        if (s.useFisheye)
        {
            Mat newCamMat;
            fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                                Matx33d::eye(), newCamMat, 1);
            fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                                             CV_16SC2, map1, map2);
        }
        else
        {
            initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                CV_16SC2, map1, map2);
        }

        for(size_t i = 0; i < s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], IMREAD_COLOR);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
        case Settings::CHESSBOARD:
        case Settings::CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; ++i)
                for (int j = 0; j < boardSize.width; ++j)
                    corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
            break;

        case Settings::ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
            break;
        default:
            break;
    }
}
//! [board_corners]
static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr, vector<Point3f>& newObjPoints,
                            float grid_width, bool release_object)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        int iFixedPoint = -1;
        if (release_object)
            iFixedPoint = s.boardSize.width - 1;
        rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                                cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                                s.flag | CALIB_USE_LU);
    }

    if (release_object) {
        cout << "New board corners: " << endl;
        cout << newObjPoints[0] << endl;
        cout << newObjPoints[s.boardSize.width - 1] << endl;
        cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << endl;
        cout << newObjPoints.back() << endl;
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, const vector<Point3f>& newObjPoints )
{
    string outputFile = s.outputFileName + s.inputCapture.GetCamName() + ".yaml";
    FileStorage fs(outputFile, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                              << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                              << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                              << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                              << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                              << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                              << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                              << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                              << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                              << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                              << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                              << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                              << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                              << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                              << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                              << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;


    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
    }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr, newObjPoints, grid_width, release_object);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr, newObjPoints);
    return ok;
}
//! [run_and_save]
