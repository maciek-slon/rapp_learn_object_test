#include <rapp/cloud/service_controller/service_controller.hpp>
#include <rapp/cloud/vision/object_detection/object_detection.hpp>

#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>
#include <rapp-robots-api/navigation/navigation.hpp>

#include <opencv2/opencv.hpp>

//#define DEBUG

#ifdef DEBUG
#define SHOW_DBG_PIC(x) cv::imshow("Out", x); cv::waitKey(-1);
#else
#define SHOW_DBG_PIC(x) ;
#endif

namespace rr = rapp::robot;

void callback(std::vector<std::string> names, std::vector<rapp::object::point> centers, std::vector<float> scores, int result) {

}

int main(int argc, char * argv[]) {
	rapp::cloud::platform_info info = {"155.207.19.229", "9001", "rapp_token"}; 
    rapp::cloud::service_controller ctrl(info);

    auto pict = std::shared_ptr<rapp::object::picture>(new rapp::object::picture(argv[1]));
    ctrl.make_call<rapp::cloud::object_detection_find_objects>(pict, 1, callback);

    
    
    rr::communication com(argc, argv);
	rr::vision vis(argc, argv);
	rr::navigation nav(argc, argv);
	
	// look straight ahead
	nav.take_predefined_posture("Stand", 0.3);
	nav.move_joint({"HeadYaw","HeadPitch"}, {0.0f, 0.0f}, 0.5);
//	nav.rest("Crouch");

	// load camera info for default/main/front-facing camera
	rr::vision::camera_info cam = vis.load_camera_info(0);
	
	// inform user
	com.text_to_speech("Give me a second...");
	
	// take picture of empty scene
	auto picture = vis.capture_image(0, rapp::robot::vision::vga4, "png");
	cv::Mat bg = cv::imdecode(picture->bytearray(), -1);
	
	// inform user
	com.text_to_speech("Put object on scene and say OK");
	std::string word;
	do {
		word = com.word_spotting( { "ok" } );
	} while (word!="ok");
	
	vis.set_camera_param(0, 11, 1);
	vis.set_camera_param(0, 12, 1);
	
	// take picture of scene with object
	picture = vis.capture_image(0, rapp::robot::vision::vga4, "png");
	cv::Mat fg = cv::imdecode(picture->bytearray(), -1);
	
	if (fg.empty() || bg.empty()) {
		std::cerr << "No input files\n";
		com.text_to_speech("Sorry, something went wrong.");
		return 0;
	}

	cv::Mat diff = (bg - fg) + (fg - bg);

	SHOW_DBG_PIC(diff);
	
	cv::cvtColor(diff, diff, CV_BGR2GRAY);

	SHOW_DBG_PIC(diff);

	cv::threshold(diff, diff, 64, 255, cv::THRESH_BINARY);

	SHOW_DBG_PIC(diff);

	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours( diff, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	cv::Rect bounding_rect;
	cv::Point center;
	float max_area = 0;
	for( size_t i = 0; i < contours.size(); i++ ) { 
		if (cv::contourArea(contours[i]) > max_area) {
			max_area = cv::contourArea(contours[i]);
			bounding_rect = cv::boundingRect( cv::Mat(contours[i]) );
			center = 0.5 * (bounding_rect.br() + bounding_rect.tl());
		}
	}

	cv::imwrite("new_object_cut.png", fg(bounding_rect));
	cv::rectangle(fg, bounding_rect.tl(), bounding_rect.br(), cv::Scalar(255, 0, 0), 2, 8, 0);
	
	SHOW_DBG_PIC(fg);

	cv::imwrite("new_object.png", fg);


	com.text_to_speech("Great, I know new object!");


	float cx = cam.K[2];
	float cy = cam.K[5];
	float fx = cam.K[0];
	float fy = cam.K[4];

	float u = center.x;
	float v = center.y;

	float a1 = -atan2(u-cx, fx);
	float a2 = atan2(v-cy, fy);

	std::cout << "K: " << fx << "," << fy << "; " << cx << "," << cy << "\n";
	std::cout << "obj: " << u << "," << v << "\n";
	std::cout << a1 << ", " << a2 << std::endl;

	nav.move_joint({"HeadYaw", "HeadPitch"}, {a1, a2}, 0.1);

	picture = vis.capture_image(0, rapp::robot::vision::vga4, "png");
	picture->save("new_object_look.png");

	// crouch and turn off all the motors
	nav.rest("Crouch");

	return 0;
}
