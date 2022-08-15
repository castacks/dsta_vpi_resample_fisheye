
#include <sstream>

#include "vpi_resample_fisheye/camera_model/camera_model_base.hpp"

using namespace mvs;

namespace mvs
{

TransformMat inverse_transform( const TransformMat& T ) {
    TransformMat inversed = TransformMat::Identity();

    const auto RT = T.block(0, 0, 3, 3).transpose();

    inversed.block(0, 0, 3, 3) = RT;
    inversed.block(0, 3, 3, 1) = -RT * T.block(0, 3, 3, 1);

    return inversed;
}

Quat ypr_2_quat(float y, float p, float r) {
    return Eigen::AngleAxis<float>( y, Eigen::Vector3f::UnitZ() )
         * Eigen::AngleAxis<float>( p, Eigen::Vector3f::UnitY() )
         * Eigen::AngleAxis<float>( r, Eigen::Vector3f::UnitX() );
}


} // namespace mvs

std::string Shape_t::string() const {
    std::stringstream ss;
    ss << "(h, w) " << h << ", " << w;
    return ss.str();
}

namespace mvs
{

std::ostream& operator << ( std::ostream& os, const Shape_t& shape ) {
    os << shape.string();
    return os;
}

} // namespace mvs



CameraModel::CameraModel(const std::string& name, float fx, float fy, float cx, float cy, const Shape_t& shape)
: name(name), fx(fx), fy(fy), cx(cx), cy(cy), shape(shape) 
{
    topic_name = "";
    frame_id = "default_camera";
    extrinsics = TransformMat::Identity();
}



CameraModel::~CameraModel()
{}


std::string CameraModel::string() const {
    std::stringstream ss;

    ss << "name = " << name << "\n"
       << "topic_name = " << topic_name << "\n"
       << "frame_id = " << frame_id << "\n"
       << "fx = " << fx << "\n"
       << "fy = " << fy << "\n"
       << "cx = " << cx << "\n"
       << "cy = " << cy << "\n"
       << "shape = " << shape << "\n"
       << "extrinsics = \n" << extrinsics;

    return ss.str();
}