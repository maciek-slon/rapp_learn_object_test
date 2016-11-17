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

int main(int argc, char * argv[]) {
	rr::communication com(argc, argv);
	rr::vision vis(argc, argv);
	rr::navigation nav(argc, argv);
	
	// look straight ahead
	nav.move_joint({"Head"}, {0.0f, 0.0f}, 0.5);

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
	float max_area = 0;
	for( size_t i = 0; i < contours.size(); i++ ) { 
		if (cv::contourArea(contours[i]) > max_area) {
			max_area = cv::contourArea(contours[i]);
			bounding_rect = cv::boundingRect( cv::Mat(contours[i]) );
		}
	}

	cv::imwrite("new_object_cut.png", fg(bounding_rect));
	cv::rectangle(fg, bounding_rect.tl(), bounding_rect.br(), cv::Scalar(255, 0, 0), 2, 8, 0);
	
	SHOW_DBG_PIC(fg);

	cv::imwrite("new_object.png", fg);


	com.text_to_speech("Great, I know new object!");

	// crouch and turn off all the motors
	nav.rest("Crouch");

	return 0;
}
