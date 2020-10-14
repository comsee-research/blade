#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>

#include "../../types.h"
#include "strategy.h"

template <typename Functors>
void make_functors(
	Functors& functors, const std::vector<IndexPair>& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode
);

template <typename Functors>
void make_functors_from_obs(
	Functors& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);
