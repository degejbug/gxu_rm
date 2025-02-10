//
// Created by Yang Shengjun on 25-1-7.
//

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>

// STD
#include <chrono>

enum {
    RED = 0,
    BLUE = 1
};

enum class ArmorType { SMALL, LARGE, INVALID };

struct Light : public cv::Rect
{
    Light() = default;
    explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
    : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
    {
        length = cv::norm(top - bottom);
        width = area / length;
        center = (top + bottom) / 2;
    }

    int color;
    cv::Point2f top, bottom;
    cv::Point2f center;
    double length;
    double width;
    float tilt_angle;
};

struct Armor
{
    Armor() = default;
    Armor(const Light & l1, const Light & l2)
    {
        if (l1.center.x < l2.center.x) {
            left_light = l1, right_light = l2;
        } else {
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
        color = l1.color;
    }

    // Light pairs part
    Light left_light, right_light;
    cv::Point2f center;

	ArmorType type;
    int color;


    // Number part
    cv::Mat number_img;
    std::string number;
    float confidence;
    std::string classfication_result;
};

struct ArmorParams
{
	double min_light_ratio;
	// light pairs distance
	double min_small_center_distance;
	double max_small_center_distance;
	double min_large_center_distance;
	double max_large_center_distance;
	// horizontal angle
	double max_angle;
};

class AssistantCameraNode : public rclcpp::Node {
public:
    AssistantCameraNode();
	~AssistantCameraNode();

	ArmorParams a;

    bool isLight(const Light & light);

    void timer_callback();
    std::vector<Light> findLights(cv::Mat& rgb_img);
    bool detectArmor(cv::Mat &rgb_img, std::vector<Light>& lights);
    void  process(cv::Mat& rgb_img);
    bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
	ArmorType isArmor(const Light & light_1, const Light & light_2);


private:
    cv::Mat img_raw;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detect_result_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    rclcpp::TimerBase::SharedPtr timer;
    cv::VideoCapture cap;

    std_msgs::msg::Bool result_msg;
	sensor_msgs::msg::Image::SharedPtr image_msg;

    // 参数
    std::string camera_device;
    int side;
    int binary_thres;
    double min_fill_ratio;	// 灯条轮廓与最小外接矩形的比值
    double min_ratio;
    double max_ratio;
    double max_angle;
};

AssistantCameraNode::AssistantCameraNode(): Node("assistant_camera_node") {
	timer = this->create_wall_timer(
		std::chrono::milliseconds(10),
		std::bind(&AssistantCameraNode::timer_callback, this)
	);  // 一秒钟检测一次，避免大量占用资源

	camera_device = "/dev/video0";
	side = RED;
	binary_thres = 200;
	min_fill_ratio = 0.8;	// 灯条轮廓与最小外接矩形的比值
	min_ratio = 0.1;
	max_ratio = 0.4;
	max_angle = 35;

    a.min_light_ratio = 0.7;
   	a.min_small_center_distance = 0.8;
    a.max_small_center_distance = 3.2;
    a.min_large_center_distance = 3.2;
    a.max_large_center_distance = 5.5;
    a.max_angle = 35.0;



	detect_result_pub_ = this->create_publisher<std_msgs::msg::Bool>("assistant_camera/result", 10);
	image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("assistant_camera/image", 10);

	cap = cv::VideoCapture(camera_device);


	// 检查摄像头是否成功打开
	if (!cap.isOpened()) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open camera device ");
	}
	else {
		RCLCPP_INFO(this->get_logger(), "Camera device opened successfully");
	}
}

void AssistantCameraNode::timer_callback()  {
	if (cap.isOpened()) {
		cv::Mat rgb_img;
		cap >> rgb_img; // 获取图像帧
		if (!rgb_img.empty()) {
			process(rgb_img);
		}
		else {
			RCLCPP_ERROR(this->get_logger(), "Failed to read frame from camera");
		}
	}
	else {
		RCLCPP_ERROR(this->get_logger(), "Camera device not opened");
		cap = cv::VideoCapture(camera_device);
	}
}

std::vector<Light> AssistantCameraNode::findLights(cv::Mat& rgb_img) {
        // 转为黑白图像
        cv::Mat gray_img;
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
        cv::Mat binary_img;
        cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);


        using std::vector;
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        vector<Light> lights;
        for (const auto & contour : contours) {
            if (contour.size() < 5) continue; 	// 忽略小于25个点的轮廓

            auto b_rect = cv::boundingRect(contour);	// 准灯条的外接矩形
            auto r_rect = cv::minAreaRect(contour);		// 准灯条的最小外接矩形

            cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);	// 创建一个和外接矩形一样大小的黑色掩膜

            // 将轮廓点转换为相对于外接矩形的坐标
            std::vector<cv::Point> mask_contour;
            for (const auto & p : contour) {
              mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
            }

        	cv::fillPoly(mask, {mask_contour}, 255);   // 在掩膜上绘制出准灯条的实心轮廓
    		std::vector<cv::Point> points;
    		cv::findNonZero(mask, points);	// 获取准灯条所有像素的坐标点
    		// points / rotated rect area
    		bool is_fill_rotated_rect = points.size() / (r_rect.size.width * r_rect.size.height) > min_fill_ratio;

    		cv::Vec4f return_param;
    		cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);

    		cv::Point2f top, bottom;
    		double angle_k;

    		if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {  // return_param[0] == 1 means k == inf	||	return_param[1] == 0 means k == 0
      		top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      		bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
      		angle_k = 0;
    		}
            else {
      			auto k = return_param[1] / return_param[0];
      			auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      			top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      			bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      			angle_k = std::atan(k) / CV_PI * 180 - 90;
      			if (angle_k > 90) {
        			angle_k = 180 - angle_k;
      			}
    		}
    	auto light = Light(b_rect, top, bottom, points.size(), angle_k);

    	if (isLight(light) && is_fill_rotated_rect) {
      		auto rect = light;
    		if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= rgb_img.rows) { // Avoid assertion failed
        		int sum_r = 0, sum_b = 0;
        		auto roi = rgb_img(rect);	// Light is a subclass of cv::Rect
        		// Iterate through the ROI
        		for (int i = 0; i < roi.rows; i++) {
          			for (int j = 0; j < roi.cols; j++) {
            			if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              			// if point is inside contour
              			sum_r += roi.at<cv::Vec3b>(i, j)[0];
              			sum_b += roi.at<cv::Vec3b>(i, j)[2];
            			}
          			}
        		}
        		// Sum of red pixels > sum of blue pixels ?
                light.color = sum_r > sum_b ? RED : BLUE;
                lights.emplace_back(light);
      		}
    	}
	}

  return lights;
}

bool AssistantCameraNode::isLight(const Light & light) {
    // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = min_ratio < ratio && ratio < max_ratio;

  bool angle_ok = light.tilt_angle < max_angle;

  bool is_light = ratio_ok && angle_ok;

  return is_light;
}

void AssistantCameraNode::process(cv::Mat& rgb_img) {
    auto lights = findLights(rgb_img);
    bool result = detectArmor(rgb_img, lights);
    if (result) {
      	result_msg.data = true;
        detect_result_pub_->publish(result_msg);

        RCLCPP_INFO(this->get_logger(), "Armor detected");
    }
    else {
    	result_msg.data = false;
    	detect_result_pub_->publish(result_msg);
        RCLCPP_INFO(this->get_logger(), "No armor detected");

        detect_result_pub_->publish(result_msg);
    }
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool AssistantCameraNode::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
	auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
	auto bounding_rect = cv::boundingRect(points);

	for (const auto & test_light : lights) {
		if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

		if (
		  bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
		  bounding_rect.contains(test_light.center)) {
			return true;
		  }
	}

	return false;
}

bool AssistantCameraNode::detectArmor(cv::Mat &rgb_img, std::vector<Light>& lights) {
	std::vector<Armor> armors;

    bool flag = false;

    // match lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_1->color != side || light_2->color != side) continue;	// 两准灯条不同色,无法配对
            if (containLight(*light_1, *light_2, lights)) continue;	// 两准灯条之间有其他灯条,无法配对

        	auto type = isArmor(*light_1, *light_2);
        	if (type != ArmorType::INVALID) {
        		auto armor = Armor(*light_1, *light_2);
        		armor.type = type;
        		armors.emplace_back(armor);
        	}
        }
    }

    for (auto armor : armors) {
    	cv::line(rgb_img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
        cv::line(rgb_img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
    	RCLCPP_INFO(this->get_logger(), armor.color == side ? "Blue armor detected" : "Red armor detected");
        if (armor.color == side) {
            flag = true;
        }
    }

	image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_img).toImageMsg();
	image_pub_->publish(*image_msg);

	return flag;
}

ArmorType AssistantCameraNode::isArmor(const Light & light_1, const Light & light_2)
{
	// Ratio of the length of 2 lights (short side / long side)
	float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length: light_2.length / light_1.length;

	bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

	// Distance between the center of 2 lights (unit : light length)
	float avg_light_length = (light_1.length + light_2.length) / 2;
	float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
	bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
							   center_distance < a.max_small_center_distance) ||
							  (a.min_large_center_distance <= center_distance &&
							   center_distance < a.max_large_center_distance);

	// Angle of light center connection
	cv::Point2f diff = light_1.center - light_2.center;
	float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
	bool angle_ok = angle < a.max_angle;

	bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

	// Judge armor type
	ArmorType type;
	if (is_armor) {
		type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
	} else {
		type = ArmorType::INVALID;
	}

	return type;
}

AssistantCameraNode::~AssistantCameraNode() {
	cap.release();
}

int main(int argc, char * argv[])
{
    // 初始化ROS 2通信
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<AssistantCameraNode>();

    // 旋转节点的执行器
    rclcpp::spin(node);

    // 关闭ROS 2通信
    rclcpp::shutdown();

    return 0;
}




