#include "odometry.hpp"

Odometry::Odometry()
{
}

Odometry::~Odometry()
{
    m_th.join();
}

void Odometry::Start()
{
    m_th = std::thread(
	[]{
	    aruco::CameraParameters params;
	    params.readFromXMLFile("./intrinsics.yml");
	    
	    aruco::MarkerDetector detector;
	    std::vector<aruco::Marker> markers;
	    const float markerSize = 0.04f;
    
	    cv::VideoCapture cap(1);
	    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
	    if(!cap.isOpened())
	    {
		exit(1);
	    }
	    cv::namedWindow("Capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

	    while(1){
		cv::Mat frame;
		cap >> frame;
		
		auto outputImage = frame.clone();
		detector.detect(frame, markers, params, markerSize);
		
		float x = 0.0f, y = 0.0f, z = 0.0f;

		for(auto&& marker : markers){
		    x += marker.Tvec.at<float>(0,0);
		    y += marker.Tvec.at<float>(0,1);
		    z += marker.Tvec.at<float>(0,2);

		    marker.draw(outputImage, cv::Scalar(0, 0, 255), 2);
		    aruco::CvDrawingUtils::draw3dCube(outputImage, marker, params);
		}
		
		x /= markers.size();
		y /= markers.size();
		z /= markers.size();

		if(markers.size() != 0)
		    std::cout << x << "\t" << y << "\t" << z << "\n";

		cv::imshow("Capture", outputImage);
		if(cv::waitKey(30) >= 0)
		{
		    break;
		}
	    }
	}
	);
    m_th.detach();
}
