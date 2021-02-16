#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

#include "../../types.h"
#include "strategy.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth(
	VirtualDepth& depth, double* cost, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	ObservationsPairingStrategy pairing = ObservationsPairingStrategy::CENTRALIZED
);

void optimize_depth_from_obs(
	VirtualDepth& depth, double* cost, //in/out
	//--------------------------------------------------------------------------	
	const BAPObservations& observations, /* (u,v,rho) */
	const PlenopticCamera& mfpc, const Image& scene
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void bruteforce_depth(
	VirtualDepth& depth, double* cost, double* sigma, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	double dmin = 2.01, double dmax = 15., double nbsample = 15.,
	ObservationsPairingStrategy pairing = ObservationsPairingStrategy::CENTRALIZED,
	bool metric = false
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void gss_depth(
	VirtualDepth& depth, double* cost, double* sigma, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	double dmin = 2.01, double dmax = 15., double tol = std::sqrt(0.001),
	ObservationsPairingStrategy pairing = ObservationsPairingStrategy::CENTRALIZED,
	bool metric = false
);
