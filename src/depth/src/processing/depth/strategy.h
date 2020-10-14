#pragma once

enum InitStrategy : std::uint16_t { RANDOM = 0, REGULAR_GRID = 1, FROM_LEFT_BORDER = 2};
enum BeliefPropagationStrategy : std::uint16_t { NONE = 0, FIRST_RING = 1, ALL_NEIGHS = 2};
enum ObservationsPairingStrategy : std::uint16_t { CENTRALIZED = 0, ALL_PAIRS = 1};
enum SearchStrategy : std::uint16_t {NONLIN_OPTIM = 0, BRUTE_FORCE = 1, GOLDEN_SECTION = 2, PROBABILISTIC = 4};

struct DepthEstimationStrategy {
	InitStrategy init 					= InitStrategy::REGULAR_GRID;
	ObservationsPairingStrategy pairing = ObservationsPairingStrategy::CENTRALIZED;
	BeliefPropagationStrategy belief	= BeliefPropagationStrategy::NONE;
	SearchStrategy search				= SearchStrategy::BRUTE_FORCE;
};

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline std::ostream& operator<<(std::ostream& os, const InitStrategy& mode)
{
	os << "Initilisation strategy = ";
	switch(mode)
	{
		case InitStrategy::RANDOM: os << "Random"; break;
		case InitStrategy::REGULAR_GRID: os << "Regular grid pattern"; break;
		case InitStrategy::FROM_LEFT_BORDER: os << "Regular pattern shifted from left"; break;
	}
	return os;
}
//******************************************************************************
inline std::ostream& operator<<(std::ostream& os, const BeliefPropagationStrategy& mode)
{
	os << "Belief propagation strategy = ";
	switch(mode)
	{
		case BeliefPropagationStrategy::NONE: os << "No propagation"; break;
		case BeliefPropagationStrategy::FIRST_RING: os << "Propagate to inner ring"; break;
		case BeliefPropagationStrategy::ALL_NEIGHS: os << "Propagate to all neighbors"; break;
	}
	return os;
}
//******************************************************************************
inline std::ostream& operator<<(std::ostream& os, const ObservationsPairingStrategy& mode)
{
	os << "Observations pairing strategy = ";
	switch(mode)
	{
		case ObservationsPairingStrategy::CENTRALIZED: os << "Centralized from ref"; break;
		case ObservationsPairingStrategy::ALL_PAIRS: os << "All possible combinations"; break;
	}
	return os;
}
//******************************************************************************
inline std::ostream& operator<<(std::ostream& os, const SearchStrategy& mode)
{
	os << "Depth search strategy = ";
	if (mode == SearchStrategy::NONLIN_OPTIM) os << "Non-linear optimization (LM)";
	if (mode == SearchStrategy::BRUTE_FORCE) os << "Brute-Force search";
	if (mode == SearchStrategy::GOLDEN_SECTION) os << "Golden-Section search (GSS)";
	if (mode == SearchStrategy::PROBABILISTIC) os << " + probabilistic estimation";
	return os;
}

//******************************************************************************
inline std::ostream& operator<<(std::ostream& os, const DepthEstimationStrategy& mode)
{
	os  << "Depth estimation strategies: " << std::endl
		<< "\t" << mode.init << std::endl
		<< "\t" << mode.pairing << std::endl
		<< "\t" << mode.belief << std::endl
		<< "\t" << mode.search;

	return os;
}




