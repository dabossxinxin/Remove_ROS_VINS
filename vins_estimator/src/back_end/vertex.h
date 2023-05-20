#pragma once

#include "eigen_types.h"

namespace backend {
	class Vertex {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit Vertex(int dimension, int local_dimension = -1);

		virtual ~Vertex();

		unsigned long Id() const;

		unsigned long OrderedId() const;

		int Dimension() const;

		int LocalDimension() const;

		virtual std::string TypeInfo() const = 0;

	protected:
		VecXd parameters;
		VecXd parameters_backup;

		int dimension;
		int local_dimension;

		unsigned long vertex_id;
		unsigned long vertex_ordered_id;

		bool is_fixed = false;
	};
}