
#define BACKWARD_HAS_DW 1

// C++ std.
// #include <filesystem> // Not supported by GCC 7.
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <thread>
#include <mutex>

#include <signal.h>

// System.
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp> // For GCC version under 8.
#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>

#include "vpi_processor.h"

// // OpenCV.
// #include <opencv2/opencv.hpp>

// // VPI.
// #include <vpi/Context.h>
// #include <vpi/Image.h>
// #include <vpi/OpenCVInterop.hpp>
// #include <vpi/Stream.h>
#include <vpi/WarpMap.h> // Not included in vpi_processor.h
// #include <vpi/algo/Remap.h>

// Other workspace packages.
#include "airlab_ros_common/ros_common.hpp"

// Local.
#include "vpi_resample_fisheye/calib/kalibr_parser.hpp"
#include "vpi_resample_fisheye/camera_model/double_sphere.hpp"

// Namespace.
// namespace fs = std::filesystem;
namespace fs = boost::filesystem;


constexpr const auto PI = boost::math::constants::pi<double>();

// Global mutex.
std::mutex g_lock;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

template<typename T>
std::vector<T> extract_number_from_string(
    const std::string& s, 
    const int expected, 
    const std::string& delimiter=",") {

    if ( expected <= 0 ) {
        std::stringstream ss;
        ss << "Exprected number must be positive. expected = " << expected << ". ";
        throw std::runtime_error(ss.str());
    }

    // Split the string.
    std::vector<std::string> splitString;
    boost::split(splitString, s, boost::is_any_of(delimiter));

    if ( splitString.size() != expected ) {
        std::stringstream ss;
        ss << "Wrong number of split strings (" << splitString.size() << "). "
           << "Expecting " << expected << " from \"" << s << "\". ";
        throw std::runtime_error(ss.str());
    }

    // Convert the strings into numbers.
    T number;
    std::stringstream ss;
    std::vector<T> numbers;

    for ( auto& fs : splitString ) {
        ss.str(""); ss.clear(); ss << fs;
        ss >> number;
        numbers.push_back(number);
    }

    return numbers;
}

static
mvs::PointMat3 get_xyz(double fov_x, double fov_y, const mvs::Shape_t& shape) {
    typedef mvs::PointMat3::Scalar Scalar_t;

    const int N = shape.size();
    
    // Angle values associated with the x and y pixel coordinates in the final pinhole camera.
    mvs::PointMat1 ax = 
        mvs::PointMat1::LinSpaced(shape.w, -1, 1) * static_cast<Scalar_t>( tan(fov_x / 2.0 / 180 * PI) );
    mvs::PointMat1 ay = 
        mvs::PointMat1::LinSpaced(shape.h, -1, 1) * static_cast<Scalar_t>( tan(fov_y / 2.0 / 180 * PI) );

    // The xyz coordinates of pixels in the pinhole camera.
    mvs::PointMat3 xyz;
    xyz.resize( 3, N );

    xyz.row(0) = 
        ax.array().matrix().replicate( 1, shape.h );
    // Eigen 3.4.
    // xyz.row(1) = 
    //     ay.array().tan().matrix().replicate( shape.w, 1 ).reshaped(1, N);

    // Eigen 3.3.
    Eigen::Matrix<Scalar_t, Eigen::Dynamic, Eigen::Dynamic> temp_row = 
        ay.array().matrix().replicate( shape.w, 1 );
    xyz.row(1) = Eigen::Map<Eigen::Matrix<Scalar_t, 1, Eigen::Dynamic>>( 
        temp_row.data(), 1, N
    );
    
    xyz.row(2) = mvs::PointMat1::Ones(1, N);
    // xyz.row(2).setZero();
    // xyz.row(2) = (mvs::PointMat1::Ones(1, N)- xyz.colwise().squaredNorm());
    // xyz.row(2) = xyz.row(2).colwise().norm();


    return xyz;
}

std::vector<mvs::DoubleSphere> read_cameras(
    const std::string& yaml_fn ) {
    
    YAML::Node calib = YAML::LoadFile(yaml_fn);

    mvs::KalibrParser parser;
    
    mvs::DoubleSphere cam0 = parser.parser_camera_node(calib["cam0"], "cam0");
    mvs::DoubleSphere cam1 = parser.parser_camera_node(calib["cam1"], "cam1");
    mvs::DoubleSphere cam2 = parser.parser_camera_node(calib["cam2"], "cam2", cam1.extrinsics);

    return { cam0, cam1, cam2 }; // Did I just make a copy?
}

namespace mvs
{

template < typename CameraModel_t >
class FisheyeResampler : public VPIProcessor {

public:
    FisheyeResampler(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_)
    : VPIProcessor(nh_, nh_private_),
      p_cam_model(nullptr),
      out_shape{200, 300}, out_fov_x(90.), out_fov_y(40.),
      prepared(false)
    {
        // Force update parent member variables.
        listenROS = true;
        publishROS = true;
        active_width  = 0;
        active_height = 0;


    }

    ~FisheyeResampler() {
        destroyResources();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!prepared) {
            ROS_WARN_STREAM("Resampler not ready yet. ");
            return;
        }

        cv_bridge::CvImagePtr _cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        processImage(_cv_ptr);
    }

    void set_camera_model(CameraModel_t* pCM) { p_cam_model = pCM; }
    void set_out_shape(const Shape_t& shape) { out_shape = shape; }
    void set_out_fov(double fov_x, double fov_y) {
        out_fov_x = fov_x;
        out_fov_y = fov_y;
    }
    void set_rotation(const Eigen::Matrix3f& r) { R = r; }

    void prepare(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_);
    void destroyResources();

protected:
    void get_remap_coordinates();
    void populate_vpi_warp_map( const cv::Mat& xx, const cv::Mat& yy, VPIWarpMap* vpi_warp_map );
    void setData(cv_bridge::CvImagePtr _cv_ptr);
    void vpi_submit();
    void processImage(cv_bridge::CvImagePtr& _cv_ptr) {
        setData(_cv_ptr);
        vpi_submit();
    }

public:
    const int PUB_BUF_LEN = 1000;
    const int SUB_BUF_LEN = 1000;

public:
    ros::Subscriber sub_fisheye;
    ros::Publisher pub_resampled;

private:
    CameraModel_t* p_cam_model;

    Shape_t out_shape;
    double out_fov_x;
    double out_fov_y;
    Eigen::Matrix3f R;

    // Re-map coordinates.
    cv::Mat xx;
    cv::Mat yy;

    // VPI stuff.
    // VPIContext ctx;
    VPIImage v_out_image;
    VPIWarpMap v_warp_map;
    VPIPayload v_warp;

    bool prepared;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename CameraModel_t >
void FisheyeResampler<CameraModel_t>::populate_vpi_warp_map( const cv::Mat& xx, const cv::Mat& yy, VPIWarpMap* vpi_warp_map ) {
    const int W = xx.cols;
    const int H = xx.rows;

    std::lock_guard<std::mutex> lock(g_lock);

    std::cout << "populat4e_vpi_warp_map: W = " << W << ", H = " << H << "\n";
    
    std::memset( vpi_warp_map, 0, sizeof(*vpi_warp_map) );
    vpi_warp_map->grid.numHorizRegions  = 1;
    vpi_warp_map->grid.numVertRegions   = 1;
    vpi_warp_map->grid.regionWidth[0]   = W;
    vpi_warp_map->grid.regionHeight[0]  = H;
    vpi_warp_map->grid.horizInterval[0] = 1;
    vpi_warp_map->grid.vertInterval[0]  = 1;
    vpiWarpMapAllocData(vpi_warp_map);

    ROS_INFO_STREAM("populate_vpi_warp_map: after vpiWarpMapAllocData(). ");

    // Sync.
    vpiStreamSync(stream);

    vpiWarpMapGenerateIdentity(vpi_warp_map);
    ROS_INFO_STREAM("populate_vpi_warp_map: after vpiWarpMapGenerateIdentity(). ");
    for ( int i = 0; i < vpi_warp_map->numVertPoints; ++i ) {
        VPIKeypoint* row = ( VPIKeypoint* ) ( ( std::uint8_t* ) vpi_warp_map->keypoints + vpi_warp_map->pitchBytes * i );
        for ( int j = 0; j < vpi_warp_map->numHorizPoints; ++j ) {
            row[j].x = xx.at<float>( i, j );
            row[j].y = yy.at<float>( i, j );
        }
    }
}

template < typename CameraModel_t >
void FisheyeResampler<CameraModel_t>::prepare(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_) {
    if ( !p_cam_model ) {
        std::string s = "p_cam_model is null. ";
        ROS_ERROR_STREAM(s);
        throw std::runtime_error(s);
    }
    
    ROS_INFO_STREAM("In prepare(). ");
    get_remap_coordinates();
    ROS_INFO_STREAM("After get_remap_coordinates(). ");

    // // VPI context.
    // vpiContextCreate(0, &ctx);
    // vpiContextSetCurrent(ctx);

    vpiImageCreate(out_shape.w, out_shape.h, VPI_IMAGE_FORMAT_BGR8, 0, &v_out_image);
    ROS_INFO_STREAM("After vpiImageCreate(). ");

    populate_vpi_warp_map(xx, yy, &v_warp_map);
    ROS_INFO_STREAM("After populate_vpi_warp_map(). ");
    vpiCreateRemap( backendType, &v_warp_map, &v_warp );
    ROS_INFO_STREAM("After vpiCreateRemap(). ");

    prepared = true;
}

template < typename CameraModel_t >
void FisheyeResampler<CameraModel_t>::destroyResources() {
    VPIProcessor::destroyResources();

    vpiPayloadDestroy(v_warp);
    vpiWarpMapFreeData(&v_warp_map);
    vpiImageDestroy(v_out_image);
    // vpiContextDestroy(ctx);
}

template < typename CameraModel_t >
void FisheyeResampler<CameraModel_t>::get_remap_coordinates() {
    // Get the xyz in the pinhole camera.
    mvs::PointMat3 xyz = get_xyz( out_fov_x, out_fov_y, out_shape );

    // Apply the rotation to the xyz in the pinhole to get the new xyz in the fisheye camera frame.
    xyz = R * xyz.eval();

    auto [ ux, uy ] = p_cam_model->project_3d_2_image_plane_separated( xyz );

    // Convert uxuy to cv::Mat.
    // // The following may cause a memory error since ux and uy will be destroyed
    // // when the end of the function is reached.
    // xx = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, ux.data() );
    // yy = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, uy.data() );
    // Copy to xx and yy.
    cv::Mat( out_shape.h, out_shape.w, CV_32FC1, ux.data() ).copyTo(xx);
    cv::Mat( out_shape.h, out_shape.w, CV_32FC1, uy.data() ).copyTo(yy);
}

template < typename CameraModel_t >
void FisheyeResampler<CameraModel_t>::setData(cv_bridge::CvImagePtr _cv_ptr) {
    VPIProcessor::setData(_cv_ptr);
 
    if (active_height != 0 || active_width != 0) {
        if ( active_height != _cv_ptr->image.rows || active_width != _cv_ptr->image.cols ) {
            ROS_ERROR_STREAM("Incoming image has a different shape. "
                << "active_height = " << active_height << ", "
                << "active_width = " << active_width << ", "
                << "_cv_ptr->image.rows = " << _cv_ptr->image.rows << ", "
                << "_cv_ptr->image.cols = " << _cv_ptr->image.cols);
            return;
        }
    }

    active_width  = _cv_ptr->image.cols;
    active_height = _cv_ptr->image.rows;
}

template < typename CameraModel_t >
void FisheyeResampler<CameraModel_t>::vpi_submit() {
    // Extract.
    vpiSubmitRemap( stream, backendType, v_warp, vImage, v_out_image, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0 );

    // Sync.
    vpiStreamSync(stream);

    // Copy the output.
    VPIImageData v_out_data;
    vpiImageLock( v_out_image, VPI_LOCK_READ, &v_out_data );

    cv::Mat out_ocv;
    vpiImageDataExportOpenCVMat(v_out_data, &out_ocv);

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time

    cv_bridge::CvImage img_bridge = 
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, out_ocv);

    pub_resampled.publish( img_bridge.toImageMsg() );

    vpiImageUnlock(v_out_image);
}

} // namespace mvs

int main(int argc, char** argv) {
    std::cout << "Hello, front_view! \n";

    ros::init(argc, argv, "vpi_resample_fisheye_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");
    signal(SIGINT, mySigIntHandler);

    GET_PARAM_DEFAULT(std::string, calib_fn, "kalibr.yaml", nh_private_)
    GET_PARAM_DEFAULT(int,         cam_idx,   0,            nh_private_)
    GET_PARAM_DEFAULT(std::string, out_size, "300, 200",    nh_private_) // Width, height.
    GET_PARAM_DEFAULT(std::string, out_fov,  "90, 40",      nh_private_) // x, y.

    // Find calibration file name to use when publishing
    std::string topic_name("");
    std::string character;
    bool found_point = false;
    for (int i = calib_fn.length()-1; i>=0; i--)
    {
        character = calib_fn[i];
        if (character.compare("/") == 0)
        {
            break;
        }
        else if ( found_point)
        {
            topic_name.push_back(calib_fn[i]);
        }
        else if (character.compare(".")==0)
        {
            found_point = true;
        }
    }
    reverse(topic_name.begin(),topic_name.end());

    auto v_out_size = extract_number_from_string<int>( out_size, 2 );
    auto v_out_fov  = extract_number_from_string<double>( out_fov, 2 );

    VPIContext ctx;
    vpiContextCreate(0, &ctx);
    vpiContextSetCurrent(ctx);

    // Create the resampler.
    mvs::FisheyeResampler<mvs::DoubleSphere> resampler(nh_, nh_private_);

    // Read the camera calibration file.
    std::vector<mvs::DoubleSphere> cameras = read_cameras( calib_fn );
    for ( const auto& camera : cameras )
        std::cout << camera << "\n";

    if ( cam_idx >= cameras.size() || cam_idx < 0 ) {
        ROS_ERROR_STREAM("Wrong cam_idx = " << cam_idx);
        return -1;
    }

    resampler.set_camera_model( &cameras[0] );

    // Set output dimensions.
    resampler.set_out_shape( { v_out_size[1], v_out_size[0] } ); // H, W.
    resampler.set_out_fov( v_out_fov[0], v_out_fov[1] ); // x, y.

    // Set re-sample rotation.
    Eigen::Matrix3f rot_mat = Eigen::Matrix3f::Zero();
    rot_mat(0, 0) = 1;
    rot_mat(1, 1) = 1;
    rot_mat(2, 2) = 1;
    // rot_mat(0, 0) = -1;
    // rot_mat(1, 2) = -1;
    // rot_mat(2, 1) = -1;
    // rot_mat(0, 0) = -1;
    // rot_mat(1, 1) = -1;
    // rot_mat(2, 2) =  1;

    resampler.set_rotation(rot_mat);

    try {
        resampler.prepare(nh_, nh_private_);
    } catch (std::exception& exc) {
        ROS_ERROR_STREAM("Exception catched during resampler.prepare(): " << exc.what());
        return -1;
    }

    // Create the subscriber.
    GET_PARAM_DEFAULT(std::string, in_topic, "/camera_0/image_raw", nh_private_)
    resampler.sub_fisheye = nh_.subscribe(in_topic, resampler.PUB_BUF_LEN, &mvs::FisheyeResampler<mvs::DoubleSphere>::imageCallback, &resampler);

    // Create the publisher.
    GET_PARAM_DEFAULT(std::string, out_topic, "/fisheye_resampler_0/"+topic_name, nh_private_)
    resampler.pub_resampled = nh_.advertise<sensor_msgs::Image>(out_topic, resampler.SUB_BUF_LEN);

    ROS_INFO_STREAM("Prepared. ");

    ros::Rate loop_rate(60);

    // Do our own spin loop
    while (!g_request_shutdown)
    {
        // Do non-callback stuff
        ros::spinOnce();
        loop_rate.sleep();
    }

    vpiContextDestroy(ctx);

    ros::shutdown();

    return 0;
}

