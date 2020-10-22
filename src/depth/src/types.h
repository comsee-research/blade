#pragma once

struct Depth { double z = 0.; };
struct VirtualDepth { double v = 0.; };

struct IndexPair {
	std::size_t k, l;
	
	IndexPair(std::size_t k_ = 0u, std::size_t l_ = 0u): k{k_}, l{l_} {}
};

using P4D = Eigen::Vector4d;

template <std::size_t N>
using PnD = Eigen::Matrix<double, N, 1>;

struct xyz 
{
	double x = 0., y = 0., z = 0.;
};

using XYZs = std::vector<xyz>;

