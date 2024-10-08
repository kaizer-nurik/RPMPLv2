#include "State.h"

base::State::State(const Eigen::VectorXf &coord_)
{
	coord = coord_;
	num_dimensions = coord.size();
	tree_idx = 0;
	idx = 0;
	d_c = -1;
	d_c_profile = std::vector<float>();
	is_real_d_c = true;
	cost = -1;
	nearest_points = nullptr;
	parent = nullptr;
	children = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
	frames = nullptr;
	skeleton = nullptr;
	enclosing_radii = nullptr;
}

base::State::~State() {}

void base::State::addChild(const std::shared_ptr<base::State> child)
{
	children->emplace_back(child);
}

namespace base 
{
	std::ostream &operator<<(std::ostream &os, const std::shared_ptr<base::State> state)
	{
		if (state->getParent() == nullptr)
			os << "q: (" << state->getCoord().transpose() << "); parent q: NONE; time: " <<state->getTime();
		else
			os << "q: (" << state->getCoord().transpose() << "); parent q: (" << state->getParent()->getCoord().transpose() <<"); time: " <<state->getTime();
		return os;
	}
}