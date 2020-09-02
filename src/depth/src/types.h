#pragma once

struct Depth { double z = 0.; };
struct VirtualDepth { double v = 0.; };

struct IndexPair {
	std::size_t k=0, l=0;
	
	IndexPair(std::size_t k_, std::size_t l_): k{k_}, l{l_} {}
};
