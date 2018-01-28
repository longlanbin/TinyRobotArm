#include "Interpolation.h"
#include <iostream>

/*****************************************************************************/
/**
* @brief Sercos III - λ�ÿ���ģʽ�£��ѿ�������ϵ�°�ֱ�߲岹�����е�����λ�� unit: deg
* @param[in] targetPoint Ŀ��λ�ã�����
* @param[in] originPoint ��ʼλ��(MCS)������
* @param[in] originACS ��ʼλ��(ACS)�� ����
* @param[in] Ts �岹����
* @param[in] maxVel ����ٶ�
* @param[in] maxAcc �����ٶ�
* @param[in] maxDecel �����ٶ�
* @param[in] maxJerk ���Ӽ��ٶ�
* @param[in] maxVelOri �����ٶ�
* @param[in] maxAccOri ���Ǽ��ٶ�
* @param[in] maxDecelOri ���Ǽ��ٶ�
* @param[in] maxJerkOri ���ǼӼ��ٶ�
* @param[out] nAglSeqPtr [ָ��]n�����λ�ò岹���ؽڿռ䣩���У�nΪ����
* @param[out] pGrpTcp [��ѡ][ָ��]�����λ�ò岹��ֱ�ǿռ䣩����
* @return ���ɹ������� RBTINTERF_NO_ERROR
* @attention ע����ڲ�����λΪ����
* @note �˶��������̫С���м��и��ݹ����
* @todo ���ٶ�����
*/
/*****************************************************************************/
ErrorID Interpolation::LineInterp(const Position_MCS_rad &targetPoint,
	const Position_MCS_rad &originPoint, const Position_ACS_rad &originACS,
	double Ts, double maxVel, double maxAcc, double maxDecel,
	double maxJerk, double maxVelOri, double maxAccOri, double maxDecelOri,
	double maxJerkOri, int polynomiType, N_AxisSeqPtr &nAglSeqPtr,
	GrpTcpSeq *pGrpTcp/* = NULL*/)
{
	if ((targetPoint.rows() != 6) || (originPoint.rows() != 6))
		return PLCOPEN_MOVE_INSTRUCT_ARG_ERROR;

	ErrorID lRet(RBTINTERF_NO_ERROR);
	AXIS_REFS_INDEX n = pRobot->numAxes();
	AXIS_REFS_INDEX axisNum;
	Position targetACS(n);

	//	����λ������δִ����ϣ��ȴ�����������״̬
	//	MC_WaitIsFree();

	//����·������
	double displacement = sqrt(pow((targetPoint[0] - originPoint[0]), 2) + pow(
		(targetPoint[1] - originPoint[1]), 2) + pow((targetPoint[2]
			- originPoint[2]), 2));

	std::deque<double> rlst;

	if (displacement > 0.2)
	{
		if (sqrt(maxAcc * displacement) < maxVel)
		{
			maxVel = sqrt(maxAcc * displacement);
			if (maxAcc / maxVel > 10)
			{
				maxVel = 10 * displacement;
				maxAcc = 100 * displacement;
			}
		}
	}
	else
	{
		if ((lRet = pRobot->calcInverseKin_RPY(targetPoint, originACS, targetACS)) != RBTINTERF_NO_ERROR)
		{
			return lRet;
		}
		else
		{
			targetACS = rad2deg(targetACS);
		}

		displacement = fabs(targetACS[0] - rad2deg(originACS[0]));
		axisNum = 0;
		for (AXIS_REFS_INDEX ai = 1; ai != n; ++ai) //2-n��
		{
			if (displacement < fabs(targetACS[ai] - rad2deg(originACS[ai])))
			{
				displacement = fabs(targetACS[ai] - rad2deg(originACS[ai]));
				axisNum = ai;
			}
		}

		if (displacement < 0.1)
		{
			return PLCOPEN_NO_MOVE_IS_NEEDED;
		}
		else
		{
			//Decare_Para decare;
			//decare = pRobot->getCurrentlyDecare();
			//maxVel = maxVel / decare.maxvel*(pRobot->axes[axisNum]->paramLmt.maxVel);
			//maxAcc = maxAcc / decare.maxacc*(pRobot->axes[axisNum]->paramLmt.maxAcc);
			//maxDecel = (pRobot->axes[axisNum]->paramLmt.maxDecel);
			if (sqrt(maxAcc * displacement) < maxVel)
			{
				maxVel = sqrt(maxAcc * displacement);
			}
			if (maxAcc / maxVel > 5)
			{
				maxVel = 5 * displacement;
				maxAcc = 25 * displacement;
			}
		}
	}
	lRet = calcInterp_5_1_5(0, 1, Ts, maxVel / displacement, maxAcc / displacement,
		maxDecel / displacement, maxJerk / displacement, polynomiType, rlst);//displacement==times
	if (lRet != RBTINTERF_NO_ERROR)
	{
		std::cout << "error: calcInterp_5_1_5" << std::endl;
		return lRet;
	}
	nAglSeqPtr.resize(n);
	for (N_AxisSeqPtrIdx idx = 0; idx != n; ++idx)
	{
		nAglSeqPtr[idx].reset(new AglSeq());
	}
	//�������Բ岹
	Matrix3 mOrigin = rpy2r(originPoint.tail(3));
	Matrix3 mTarget = rpy2r(targetPoint.tail(3));

	Vector3 pOrigin = originPoint.head(3);
	Vector3 pTarget = targetPoint.head(3);
	Vector3 pr;
	dmatrix Tr(4, 4);
	Position_MCS_rad posr(6);
	Position_ACS_rad posACS(n);
	Position_ACS_rad pLast(n);
	dQuaternion qr;
	dQuaternion qTarget(mTarget);
	dQuaternion qOrigin(mOrigin);

	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r = *rit;
		qr = qOrigin.slerp(r, qTarget);
		pr = pOrigin*(1 - r) + r*pTarget;
		Tr << qr.toRotationMatrix(), pr, dmatrix::Zero(1, 3), 1;

		posr = tr2MCS(Tr);

		//�ж���ֱ������ϵ���Ƿ���
		rectLimit = pRobot->RobotRangeLimit(posr);
		if (rectLimit != RANGE_NOOVERLIMIT)
		{
			std::cout << "error: RobotRangeLimit!" << std::endl;
			return rectLimit;
		}

		if (rit == rlst.begin())
		{
			pLast = originACS;
		}
		lRet = pRobot->calcInverseKin_RPY(posr, pLast, posACS);
		speedLimit = pRobot->RobotSpeedLimit(pLast, posACS, Ts, n);
		if (speedLimit != SPEED_NOOVERLIMIT)
		{
			std::cout << "error: RobotSpeedLimit!" << std::endl;
			return speedLimit;
		}
		if (lRet != RBTINTERF_NO_ERROR)
		{
			if (pGrpTcp)
				pGrpTcp->clear();
			std::cout << "error: calcInverseKin_RPY!" << std::endl;
			return lRet;
		}
		else
		{
			if (pGrpTcp)
				pGrpTcp->push_back(posr);//��¼tcp
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) //1-n��
			{
				nAglSeqPtr[ai]->push_back(rad2deg(posACS[ai]));
			}
			pLast = posACS;
		}
	}
	return RBTINTERF_NO_ERROR;
}
