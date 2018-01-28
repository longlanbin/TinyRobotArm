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
* @brief ��ļ���ֵ
* @note  ���в�����ֵ�Ǵӻ����˵Ĳ����ļ��ж�ȡ��
* @see Axis
*/
typedef struct tagAXIS_LIMIT
{
	double posSWLimit; /**< @brief ��������λ��(��ֵ), unit: deg */
	double negSWLimit; /**< @brief ��������λ��(��ֵ), unit: deg */
	double maxVel; /**< @brief �����ٶ�, unit: deg/s */
	double maxRdcVel; /**< @brief �����ٽ��ٶ�, unit: deg/s */
	double maxAcc; /**< @brief ���Ǽ��ٶ�, unit: deg/(s^2) */
	double maxDecel; /**< @brief ���Ǽ��ٶ�, unit: deg/(s^2) */
	double maxRotSpeed; /**< @brief ����������ת��, unit: deg/s */
	double maxDeRotSpeed; /**< @brief �����������ٶ�, unit: deg/s */
} AXIS_LIMIT;

typedef struct tagJogSpeed
{
	double MaxSpeed;
	double MaxAcc;
} JogSpeed;

/**
* @class	tagAxis_DH_INIT_DATA Axis.h
* @brief	��ĳ�ʼ������
* @note    ���в�����ֵ�Ǵӻ����˵Ĳ����ļ��ж�ȡ��
* @see Axis
*/
typedef struct tagAxis_DH_INIT_DATA
{
	int AxisNumber; /**< @brief ���� */
	DenavitHartenbergParameters DH;/**< @brief �ؽڵ�DH���� */
	AXIS_LIMIT _limit; /**< @brief �ؽڽǶȼ���ֵ */
	double _offset; /**< @brief �ؽڽǶȳ�ʼֵƫ�� */
	double _reducRatio;
	int _direction;
	int _encoderResolution;
	double _singleTurn;
	JogSpeed _JogJointSpeed;
} AXIS_DH_INIT_PARAM;

/**
* @class	Axis Axis.h
* @brief	���ڼ�¼��Ĳ�����������
* ����Ϊ����������ͣ����ڴ�ŵ���ģ�Ͳ������˶��岹�����Լ�������� ( �� Drive )��
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

	int AxisNumber; /**< @brief ���� */
	DenavitHartenbergJoint DHJ; /**< @brief DH�����ṹ */
	AXIS_LIMIT paramLmt; /**< @brief �ؽڼ���ֵ */
	double Offset; /**< @brief �ؽڽǶȳ�ʼֵƫ�� */
	double reducRatio;
	int direction;
	int encoderResolution;
	JogSpeed JogJointSpeed;
};



