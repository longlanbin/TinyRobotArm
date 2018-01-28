#pragma once

#include "AbstractRobot.h"	
#include "Axis.h"
#include "RbtInterfStatus.h"

extern size_t jointLimitNum;
extern ErrorID rectLimit;
extern ErrorID speedLimit;

class AxesGroup;
typedef std::shared_ptr<AxesGroup> AXES_GROUP_REF;
typedef int IDENT_IN_GROUP_REF;

typedef struct tagAXIS_Couple_Coe
{
	double Couple_Coe_1_2;
	double Couple_Coe_2_3;
	double Couple_Coe_3_2;
	double Couple_Coe_3_4;
	double Couple_Coe_4_5;
	double Couple_Coe_4_6;
	double Couple_Coe_5_6;
} AXIS_Couple_Coe;

typedef struct tagRANGE_Limit
{
	double maxX;
	double maxY;
	double maxZ;
	double minX;
	double minY;
	double minZ;
} RANGE_Limit;

class SerialRobotModel:
	public AbstractRobot
{
public:
	SerialRobotModel();
	virtual	~SerialRobotModel()=0;
	AXIS_REFS_INDEX numAxes() const
	{
		return axes.size();
	}

	virtual	void setRobotModelParameter() = 0;

	//�ṩ���壬�ɸ��ô��룬��Ϊ�󲿷ִ��������������㷨һ����
	//������Ϊ���麯����ͬʱ���ṩ���壬��������ᱨ��
	//virtual size_t calcForwardKin(const dvector& posACS, dvector6& posMCS) /*= 0*/;
	virtual ErrorID calcForwardKin(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS);

	//virtual size_t calcInverseKin_RPY(const dvector6& posMCS, const dvector& posLast, dvector& posACS) const = 0;  //���Բ��ṩ����
	virtual ErrorID calcInverseKin_RPY(const Position_MCS_rad& posMCS,
									   const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const = 0;

	 ErrorID calcTransMatrix(const Position_ACS_rad& posACS, dmatrix& transMatrix);




	//ErrorID calcInverseKin_Trans_6s(const dmatrix& transMatrix, const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const;

	double calcRealAngle(double curAng, double candidateAng1, double candidateAng2) const;
	bool is_Near_Odd(double theta_rad, double agl_offset_rad, bool is_Ntimes_90deg_Odd) const;

	/**
	* ����ĺ��������ж�ֱ�����귶Χ�Ƿ���
	* ����ֵ��	RANGE_OVERLIMIT_X��x�ᳬ�ޣ�
	* 			RANGE_OVERLIMIT_Y��y�ᳬ�ޣ�
	* 			RANGE_OVERLIMIT_Z��z�ᳬ�ޣ�
	* 			RANGE_NOOVERLIMIT���޳���
	*/
	ErrorID RobotRangeLimit(const Position_MCS_rad& posMCS);

	//�жϻ����˸����Ƿ���
	ErrorID RobotSpeedLimit(const Position_ACS_deg& pos, const Position_ACS_deg& posACS, double Ts, int num);

	N_AXIS_REFS axes;
	dmatrix transformationMatrix;
	dmatrix baseMatrix;
	Matrix4 toolMatrix;
	AXIS_Couple_Coe _coupleCoe;
	RANGE_Limit range_limit;

	const double ODD_HANDLE_MARGIN;//todo: ������ж�ԣ�ȣ��ݶ�Ϊ0.001���������Ը���
	const double ODD_MARGIN;
};

