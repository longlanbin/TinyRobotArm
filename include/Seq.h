#pragma once
#include "RobotData.h"
#include <memory>            //use std::shared_ptr instead of boost::shared_ptr
#include <deque>
#include <vector>

typedef /*Seq*/std::deque<double> AglSeq; /**< @brief �Ƕȶ��� */
typedef std::shared_ptr<AglSeq> AxisSeqPtr; /**< @brief �Ƕȶ���ָ��*/
typedef std::vector<AxisSeqPtr> N_AxisSeqPtr; /**< @brief һ��Ƕȶ���ָ�� */
typedef N_AxisSeqPtr::size_type N_AxisSeqPtrIdx; /**< @brief �Ƕȶ���ָ�������� */
typedef /*Seq*/std::deque<Position_ACS_deg, Eigen::aligned_allocator<Position_ACS_rad> > GrpAglSeq;
typedef /*Seq*/std::deque<Position_MCS_deg, Eigen::aligned_allocator<Position_MCS_rad> > GrpTcpSeq;