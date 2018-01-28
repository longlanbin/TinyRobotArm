#pragma once
#include "EigenTypes.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define rad2deg(r) ((r)*180/M_PI)
#define deg2rad(d) ((d)*M_PI/180)

//位置
typedef dvector Position;
typedef dvector Position_ACS;
typedef dvector Position_ACS_rad; /**< unit: radian */
typedef dvector Position_ACS_deg; /**< unit: degree */
typedef dvector Position_MCS;
typedef dvector Position_MCS_rad; /**< unit: mm, radian */
typedef dvector Position_MCS_deg; /**< unit: mm, degree */
								  //速度
typedef dvector Velocity;
typedef dvector Velocity_ACS;
typedef dvector Velocity_ACS_rad; /**< unit: radian/s */
typedef dvector Velocity_ACS_deg; /**< unit: degree/s */
typedef dvector Velocity_MCS;
typedef dvector Velocity_MCS_rad; /**< unit: mm/s, radian/s */
typedef dvector Velocity_MCS_deg; /**< unit: mm/s, degree/s */
								  //加速度
typedef dvector Acceleration;
typedef dvector Acceleration_ACS;
typedef dvector Acceleration_ACS_rad; /**< unit: radian/(s^2) */
typedef dvector Acceleration_ACS_deg; /**< unit: degree/(s^2) */
typedef dvector Acceleration_MCS;
typedef dvector Acceleration_MCS_rad; /**< unit: mm/(s^2), radian/(s^2) */
typedef dvector Acceleration_MCS_deg; /**< unit: mm/(s^2), degree/(s^2) */
									  //DH

typedef Vector3 XYZ; /**< @brief x,y,z */
typedef Vector3 RPY; /**< @brief x,y,z order */
struct DenavitHartenbergParameters;
struct DenavitHartenbergMatrix;
inline RPY tr2rpy(dmatrix m);/**<x,y,z order, 弧度 */
inline Position_MCS_rad tr2MCS(dmatrix m);/**<x,y,z order, 弧度 */
inline dmatrix rpy2r(RPY rpy);/**<x,y,z order, 弧度 */
inline dmatrix rpy2tr(Position_MCS_rad posMCS);/**<x,y,z order, 弧度 */
inline Position_MCS_rad posMCS_rad2deg(Position rpy);
inline Position_MCS_deg posMCS_deg2rad(Position rpy);


/**
* @brief  将矩阵转化为rpy
* @param[in] m 位姿矩阵
* @return rpy
*/
inline RPY tr2rpy(dmatrix m)
{
	RPY rpy;
	rpy(0) = atan2(-m(1, 2), m(2, 2));
	double sr = sin(rpy(0, 0));
	double cr = cos(rpy(0, 0));
	rpy(1) = atan2(m(0, 2), cr*m(2, 2) - sr*m(1, 2));
	rpy(2) = atan2(-m(0, 1), m(0, 0));
	return rpy;
}

/**
* @brief  将矩阵转化为机器人空间中tcp点位姿
* @param[in] m 位姿矩阵
* @return posMCS 是tcp点在空间中的位置和姿态
*/
inline Position_MCS_rad tr2MCS(dmatrix m)
{
	Position_MCS_rad posMCS(6);
	posMCS << m.block(0, 3, 3, 1), tr2rpy(m);
	return posMCS;
}

/**
* @brief  将rpy转换为4*4矩阵中的姿态的3*3矩阵
* @param[in] rpy TCP的姿态
* @return 4*4矩阵中的姿态的3*3矩阵
*/
inline dmatrix rpy2r(RPY rpy)
{
	return Eigen::AngleAxisd(rpy(0), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitZ()).toRotationMatrix();
}

/**
* @brief  将TCP的位姿转换为4*4位姿矩阵
* @param[in]  posMCS TCP的位姿
* @return tr 4*4的位姿矩阵
*/
inline dmatrix rpy2tr(Position_MCS_rad posMCS)
{
	dmatrix tr(4, 4);
	tr << rpy2r(posMCS.tail(3)), posMCS.head(3), dmatrix::Zero(1, 3), 1;
	return tr;
}
inline Position_MCS_rad posMCS_deg2rad(Position_MCS rpy)
{
	rpy.block(3, 0, 3, 1) = deg2rad(rpy.block(3, 0, 3, 1));
	return rpy;
}
inline Position_MCS_deg posMCS_rad2deg(Position_MCS rpy)
{
	rpy.block(3, 0, 3, 1) = rad2deg(rpy.block(3, 0, 3, 1));
	return rpy;
}
/***********************************************************************/
/**
* @class DenavitHartenbergParameters RobotData.h
* @brief DH参数
*
*   Initializes a new instance of the <see cref="DenavitHartenbergParameters"/> class.
* <param name="alpha">Angle (in radians) of the Z axis relative to the last joint.</param>
* <param name="radius">Length or radius of the joint.</param>
* <param name="theta">Angle (in radians) of the X axis relative to the last joint.</param>
* <param name="offset">Offset along Z axis relatively to the last joint.</param>
*/
/***********************************************************************/
struct DenavitHartenbergParameters
{
	DenavitHartenbergParameters() :
		Alpha(0), A(0), Theta(0), D(0)
	{
	}
	DenavitHartenbergParameters(double alpha, double a, double theta, double d) :
		Alpha(alpha), A(a), Theta(theta), D(d)
	{
	}
	double Alpha; /**< @brief Angle in radians about common normal, from old <c>z</c> axis to the new <c>z</c> axis. */
	double A; /**< @brief Length of the joint (also known as <c>a</c>). */
	double Theta; /**< @brief Angle in radians about previous <c>z</c>, from old <c>x</c> to the new <c>x</c>. */
	double D; /**< @brief Offset along previous <c>z</c> to the common normal (also known as <c>d</c>). */
};

/***********************************************************************/
/**
* @class DenavitHartenbergMatrix RobotData.h
* @brief DH参数矩阵
*/
/***********************************************************************/
struct DenavitHartenbergMatrix
{
	DenavitHartenbergMatrix() :
		Transform(4, 4), X(4, 4), Z(4, 4)
	{
	}

	dmatrix Transform; /**< @brief Gets or sets the transformation matrix T (as in T = Z * X). */
	dmatrix X; /**< @brief Gets or sets the matrix regarding X axis transformations. */
	dmatrix Z; /**< @brief Gets or sets the matrix regarding Z axis transformations. */
			   /**
			   * @brief Executes the transform calculations (T = Z*X).
			   *
			   * Calling this method also updates the Transform property.
			   * @return Transform matrix T.
			   */
	void Compute(DenavitHartenbergParameters &parameters)
	{
		// Calculate Z with Z = TranslationZ(d).RotationZ(theta)
		Z << Eigen::AngleAxisd(parameters.Theta, Vector3::UnitZ()).toRotationMatrix(), (dmatrix(3, 1) << 0.0, 0.0, parameters.D).finished(), dmatrix::Zero(
			1, 3), 1;
		// Calculate X with X = TranslationX(radius).RotationZ(alpha)
		X << Eigen::AngleAxisd(parameters.Alpha, Vector3::UnitX()).toRotationMatrix(), (dmatrix(3, 1) << parameters.A, 0.0, 0.0).finished(), dmatrix::Zero(
			1, 3), 1;
		// Calculate the transform with T=Z.X
		Transform = Z*X;
	}
};