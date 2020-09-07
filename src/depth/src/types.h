#pragma once

struct Depth { double z = 0.; };
struct VirtualDepth { double v = 0.; };

struct IndexPair {
	std::size_t k, l;
	
	IndexPair(std::size_t k_ = 0u, std::size_t l_ = 0u): k{k_}, l{l_} {}
};
