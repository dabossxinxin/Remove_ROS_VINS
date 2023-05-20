#include "vertex.h"

namespace backend {
	unsigned long global_vertex_id = 0;

	Vertex::Vertex(int num_dimension, int local_dimension) 
	{
		parameters.resize(num_dimension, 1);
		local_dimension = local_dimension > 0 ? local_dimension : num_dimension;
		vertex_id = global_vertex_id++;
	}

	Vertex::~Vertex() {}

	int Vertex::Dimension() const 
	{
		return dimension;
	}

	int Vertex::LocalDimension() const 
	{
		return local_dimension;
	}

	unsigned long Vertex::Id() const
	{
		return vertex_id;
	}

	unsigned long Vertex::OrderedId() const
	{
		return vertex_ordered_id;
	}
}