#pragma once

#include "DenavitHartenbergJoint.h"
#include <memory>
#include <deque>

class Axis;
typedef std::shared_ptr<Axis> AXIS_REF;
typedef std::deque<AXIS_REF> N_AXIS_REFS;
typedef std::deque<AXIS_REF>::size_type AXIS_REFS_INDEX;
/**
* @class tagAXIS_LIMIT Axis.h
* @brief 轴的极限值
* @note  所有参数的值是从机器人的参数文件中读取的
* @see Axis
*/
typedef struct tagAXIS_LIMIT
{
	double posSWLimit; /**< @brief 正向最大角位移(正值), unit: deg */
	double negSWLimit; /**< @brief 反向最大角位移(负值), unit: deg */
	double maxVel; /**< @brief 最大角速度, unit: deg/s */
	double maxRdcVel; /**< @brief 最大减速角速度, unit: deg/s */
	double maxAcc; /**< @brief 最大角加速度, unit: deg/(s^2) */
	double maxDecel; /**< @brief 最大角减速度, unit: deg/(s^2) */
	double maxRotSpeed; /**< @brief 正向最大输出转速, unit: deg/s */
	double maxDeRotSpeed; /**< @brief 反向最大输出速度, unit: deg/s */
} AXIS_LIMIT;

typedef struct tagJogSpeed
{
	double MaxSpeed;
	double MaxAcc;
} JogSpeed;

/**
* @class	tagAxis_DH_INIT_DATA Axis.h
* @brief	轴的初始化参数
* @note    所有参数的值是从机器人的参数文件中读取的
* @see Axis
*/
typedef struct tagAxis_DH_INIT_DATA
{
	int AxisNumber; /**< @brief 轴编号 */
	DenavitHartenbergParameters DH;/**< @brief 关节的DH参数 */
	AXIS_LIMIT _limit; /**< @brief 关节角度极限值 */
	double _offset; /**< @brief 关节角度初始值偏移 */
	double _reducRatio;
	int _direction;
	int _encoderResolution;
	double _singleTurn;
	JogSpeed _JogJointSpeed;
} AXIS_DH_INIT_PARAM;

/**
* @class	Axis Axis.h
* @brief	用于记录轴的参数配置数据
* 本类为单轴的类类型，用于存放单轴模型参数、运动插补序列以及电机参数 ( 见 Drive )。
* @see Drive
*/
class Axis
{
public:
	Axis()
	{
	};
	~Axis()
	{
	};

	int AxisNumber; /**< @brief 轴编号 */
	DenavitHartenbergJoint DHJ; /**< @brief DH参数结构 */
	AXIS_LIMIT paramLmt; /**< @brief 关节极限值 */
	double Offset; /**< @brief 关节角度初始值偏移 */
	double reducRatio;
	int direction;
	int encoderResolution;
	JogSpeed JogJointSpeed;
};



