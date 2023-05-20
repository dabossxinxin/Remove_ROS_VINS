#pragma once

#include "eigen_types.h"

namespace backend {
	class LossFunction {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		
		virtual ~LossFunction() {}

		virtual void Compute(double ee, Eigen::Vector3d& rho) const = 0;
	};
}