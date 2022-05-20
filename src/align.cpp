#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.h>
#include <eigen3/Eigen/Core>

using std::placeholders::_1;


template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
{
    if (!(src.Flags & Eigen::RowMajorBit))
    {
        cv::Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
            (void*)src.data(), src.stride() * sizeof(_Tp));
        cv::transpose(_src, dst);
    }
    else
    {
        cv::Mat _src(src.rows(), src.cols(), cv::DataType<_Tp>::type,
            (void*)src.data(), src.stride() * sizeof(_Tp));
        _src.copyTo(dst);
    }
}

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void cv2eigen( const cv::Mat& src,
               Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst )
{
    CV_DbgAssert(src.rows == _rows && src.cols == _cols);
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else if( src.cols == src.rows )
        {
            src.convertTo(_dst, _dst.type());
            transpose(_dst, _dst);
        }
        else
            cv::Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}

template<typename _Tp>
void cv2eigen( const cv::Mat& src,
               Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic>& dst )
{
    dst.resize(src.rows, src.cols);
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
             dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else if( src.cols == src.rows )
        {
            src.convertTo(_dst, _dst.type());
            transpose(_dst, _dst);
        }
        else
            cv::Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}

template<typename _Tp>
void cv2eigen( const cv::Mat& src,
               Eigen::Matrix<_Tp, Eigen::Dynamic, 1>& dst )
{
    CV_Assert(src.cols == 1);
    dst.resize(src.rows);

    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else
            cv::Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}

template<typename _Tp>
void cv2eigen( const cv::Mat& src,
               Eigen::Matrix<_Tp, 1, Eigen::Dynamic>& dst )
{
    CV_Assert(src.rows == 1);
    dst.resize(src.cols);
    if( !(dst.Flags & Eigen::RowMajorBit) )
    {
        cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        if( src.type() == _dst.type() )
            transpose(src, _dst);
        else
            cv::Mat(src.t()).convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
    else
    {
        cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
                 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
        src.convertTo(_dst, _dst.type());
        CV_DbgAssert(_dst.data == (uchar*)dst.data());
    }
}


class Align : public rclcpp::Node
{
  public:
    Align()
    : Node("align")
    {
      m_sub_ir = this->create_subscription<sensor_msgs::msg::Image>(
        "/thermal/camera/image_rect", 10,
        std::bind(&Align::ir_callback, this, _1));

      m_sub_ir_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/thermal/camera/camera_info", 10,
        std::bind(&Align::ir_info_callback, this, _1));

      m_sub_depth = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", 10,
        std::bind(&Align::depth_callback, this, _1));

      m_sub_depth_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", 10,
        std::bind(&Align::depth_info_callback, this, _1));

      m_pub_align = this->create_publisher<sensor_msgs::msg::Image>(
        "/thermal/camera/depth_aligned", 10);
    }

  private:
    // publications
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_align;

    // subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub_ir, m_sub_depth;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_sub_ir_info, m_sub_depth_info;

    // member data
    sensor_msgs::msg::CameraInfo::SharedPtr m_ir_info{}, m_depth_info{};
    sensor_msgs::msg::Image::SharedPtr m_ir_image;
    sensor_msgs::msg::Image::SharedPtr m_depth_image;

    // callbacks
    void ir_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

      RCLCPP_INFO(this->get_logger(), "I heard an ir image");
      m_ir_image = msg;

      try {
        auto &p = m_depth_info.get()->p;
        Eigen::Matrix3d K_depth;
        K_depth << p[0], p[1], p[2],
          p[4], p[5], p[6],
          p[8], p[9], p[10];
        std::cout << K_depth << std::endl;

        p = m_ir_info.get()->p;
        Eigen::Matrix3d K_ir;
        K_ir << p[0], p[1], p[2],
          p[4], p[5], p[6],
          p[8], p[9], p[10];
        std::cout << K_ir << std::endl;

        using namespace Eigen;
        cv::Mat img = cv_bridge::toCvCopy(m_depth_image, m_depth_image->encoding)->image;
        Matrix<uint16_t,Dynamic,Dynamic> b;
        cv::Mat c;
        cv2eigen(img, b);
        std::cout << "eigen size: " << b.rows() << "x" << b.cols() << "  size: " << b.size() << std::endl;
        eigen2cv(b, c);
      
        // this is where publication occurs
        auto cvimg = cv_bridge::CvImage();
        cv_bridge::CvImage img_bridge;
        sensor_msgs::msg::Image img_msg;
        std_msgs::msg::Header header;
        std::cout << m_depth_image->encoding << std::endl;
        img_bridge = cv_bridge::CvImage(msg->header, m_depth_image->encoding, c);
        //img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO16, c);
        img_bridge.toImageMsg(img_msg);
        m_pub_align->publish(img_msg);
      }
      catch(std::exception & e) {
        RCLCPP_INFO(this->get_logger(), e.what());
      }
    }

    void ir_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard an ir info image");
      m_ir_info = msg;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard a depth image");
      m_depth_image = msg;
    }

    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard a depth info image");
      m_depth_info = msg;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Align>());
  rclcpp::shutdown();
  return 0;
}
