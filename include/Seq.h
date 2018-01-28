#pragma once
#include "RobotData.h"
#include <memory>            //use std::shared_ptr instead of boost::shared_ptr
#include <deque>
#include <vector>

typedef /*Seq*/std::deque<double> AglSeq; /**< @brief 角度队列 */
typedef std::shared_ptr<AglSeq> AxisSeqPtr; /**< @brief 角度队列指针*/
typedef std::vector<AxisSeqPtr> N_AxisSeqPtr; /**< @brief 一组角度队列指针 */
typedef N_AxisSeqPtr::size_type N_AxisSeqPtrIdx; /**< @brief 角度队列指针索引号 */
typedef /*Seq*/std::deque<Position_ACS_deg, Eigen::aligned_allocator<Position_ACS_rad> > GrpAglSeq;
typedef /*Seq*/std::deque<Position_MCS_deg, Eigen::aligned_allocator<Position_MCS_rad> > GrpTcpSeq;