#include "odometry.hpp"

#include <sys/time.h>

Odometry::Odometry(float *camera_theta)
{
    m_camera_theta = camera_theta;
    m_x = 0.0f;
    m_y = 0.0f;
    m_z = 0.0f;
}

Odometry::~Odometry()
{
    m_th.join();
}

int sgn(float x){return x == 0 ? 0 : (x > 0 ? 1 : -1);}

Position Rotate(Position origin, float phi)
{
    Position ret;
    ret.x = cos(phi) * origin.x - sin(phi) * origin.y;
    ret.y = sin(phi) * origin.x + cos(phi) * origin.y;
    ret.theta = origin.theta - phi;

    /*std::cout << origin.x << "\t"
	      << origin.y << "\t"
	      << origin.theta << "\t"
	      << ret.x << "\t"
	      << ret.y << "\t"
	      << ret.theta << "\t"
	      << phi << "\n";*/
    return ret;
}

void Odometry::Start()
{
    m_th = std::thread(
	[this]{
	    aruco::CameraParameters params;
	    params.readFromXMLFile("./intrinsics.yml");
	    
	    aruco::MarkerDetector detector;
	    std::vector<aruco::Marker> markers;
	    const float markerSize = 0.156f;
    
	    cv::VideoCapture cap(1);
	    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
	    if(!cap.isOpened())
	    {
		exit(1);
	    }
	    cv::namedWindow("Capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

	    struct timeval t, old_t;
	    float old_x = 0.0f, old_y = 0.0f, old_z = 0.0f;
	    int duration;
	    while(1){
		cv::Mat frame;
		cap >> frame;
		auto outputImage = frame.clone();
		detector.detect(frame, markers, params, markerSize);

		if(markers.size() != 0){
		    old_t = t;
		    gettimeofday(&t, NULL);
		    duration = 1000000 * (t.tv_sec - old_t.tv_sec) + t.tv_usec - old_t.tv_usec;
		
		    float x = 0.0f, y = 0.0f, z = 0.0f;

		    for(auto&& marker : markers){
			x += marker.Tvec.at<float>(0,0);
			y += marker.Tvec.at<float>(0,1);
			z += marker.Tvec.at<float>(0,2);
			m_marker_vec[marker.id] = Rotate({
			    marker.Tvec.at<float>(0,0),
			    marker.Tvec.at<float>(0,2),
			    - marker.Rvec.at<float>(0,2) * sgn(marker.Rvec.at<float>(0,0))
				}, *m_camera_theta);
			marker.draw(outputImage, cv::Scalar(0, 0, 255), 2);
			aruco::CvDrawingUtils::draw3dCube(outputImage, marker, params);
		    }

		    x /= markers.size();
		    y /= markers.size();
		    z /= markers.size();

		    m_is_set = true;

		    old_x = x; old_y = y; old_z = z;		    
		}else{
		    m_is_set = false;
		}

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
