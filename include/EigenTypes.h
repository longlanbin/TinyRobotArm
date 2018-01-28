#pragma once

#ifdef random
#undef random
#endif

#define EIGEN_DONT_ALIGN          /**< @brief ÆÁ±ÎEIGEN¿âµÄ¶ÏÑÔ¹¦ÄÜ */
#define EIGEN_DONT_VECTORIZE      /**< @brief ÆÁ±ÎEIGEN¿âµÄ¶ÑÕ»¶ÔÆëÒªÇó */
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT     /**< @brief ÆÁ±ÎEIGEN¿âµÄ¶ÑÕ»¶ÔÆëÒªÇó */
#include <Eigen/Eigen>
#define EIGEN_DONT_ALIGN_STATICALLY


typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;
typedef Eigen::MatrixXd dmatrix;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::VectorXd dvector;
typedef Eigen::Matrix<double, 6, 1> dvector6;
typedef dmatrix::Index dmIndex;
typedef dvector::Index dvIndex;
typedef Eigen::Quaternion<double> dQuaternion;