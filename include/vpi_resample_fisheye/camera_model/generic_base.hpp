#ifndef __CAMERA_MODEL_GENERIC_BASE_HPP__
#define __CAMERA_MODEL_GENERIC_BASE_HPP__

#include <Eigen/Core>

template<typename Scalar>
class BaseCameraClass
{
    public:
        BaseCameraClass(){}
        virtual bool Read(const char* yaml_path, std::string* error_reason = nullptr){return true;}
        
        
        typedef Eigen::Matrix<Scalar, 2, 1> PixelT;
        // typedef Eigen::Matrix<Scalar, 3, 1> PointT;

        template <typename Derived>
        bool Project(const Eigen::MatrixBase<Derived>& point,PixelT* pixel){return true;}

        // template <typename Derived>
        // bool ProjectWithInitialEstimate(const Eigen::MatrixBase<Derived>& point,PixelT* pixel);

        // template <typename Derived, typename >
        // bool ProjectWithJacobian(const Eigen::MatrixBase<Derived>& point,PixelT* pixel, Eigen::Matrix<Scalar, 2, 3>* jacobian, Scalar numerical_diff_delta = 1e-4);

        // template <typename Derived>
        // bool ProjectWithJacobianAndInitialEstimate(const Eigen::MatrixBase<Derived>& point, PixelT* pixel, Eigen::Matrix<Scalar, 2, 3>* jacobian,Scalar numerical_diff_delta = 1e-4); 

        // template <typename Derived>
        // bool Unproject(const Eigen::MatrixBase<Derived>& pixel, PointT* direction);

        // template <typename Derived>
        // bool UnprojectWithJacobian(const Eigen::MatrixBase<Derived>& pixel, PointT* direction, Eigen::Matrix<Scalar, 3, 2>* jacobian);
        /// Returns the width of the camera images.
        inline int width() const { return m_width; }
        
        /// Returns the height of the camera images.
        inline int height() const { return m_height; }
        
        /// Returns the left x coordinate of the calibrated image area rectangle.
        inline int calibration_min_x() const { return m_calibration_min_x; }
        
        /// Returns the top y coordinate of the calibrated image area rectangle.
        inline int calibration_min_y() const { return m_calibration_min_y; }
        
        /// Returns the right x coordinate of the calibrated image area rectangle.
        inline int calibration_max_x() const { return m_calibration_max_x; }
        
        /// Returns the bottom y coordinate of the calibrated image area rectangle.
        inline int calibration_max_y() const { return m_calibration_max_y; }

        int m_width;
        int m_height;
        int m_calibration_min_x;
        int m_calibration_min_y;
        int m_calibration_max_x;
        int m_calibration_max_y;
        int m_grid_width;
        int m_grid_height;
};

#endif