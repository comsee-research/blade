#pragma once

#include "geometry/depth/RawCoarseDepthMap.h"

constexpr double AUTOMATIC_FILTER_SIZE = -1.;

//******************************************************************************
//******************************************************************************
RawCoarseDepthMap median_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_median_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);

RawCoarseDepthMap mean_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_mean_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);

RawCoarseDepthMap minmax_filter_depth(const RawCoarseDepthMap& dm, double min, double max); 
void inplace_minmax_filter_depth(RawCoarseDepthMap& dm, double min, double max); 

//******************************************************************************
//******************************************************************************
RawCoarseDepthMap erosion_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_erosion_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 
 
//******************************************************************************
//******************************************************************************
/* E(I,Z)= min I(Z) */ 
RawCoarseDepthMap morph_erosion_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_erosion_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* D(I,Z)= max I(Z) */ 
RawCoarseDepthMap morph_dilation_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_dilation_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* O(I,Z)= D(E(I,Z),Z) */ 
RawCoarseDepthMap morph_opening_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_opening_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* C(I,Z)= E(D(I,Z),Z) */ 
RawCoarseDepthMap morph_closing_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_closing_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* S(I,Z)= C(O(I,Z),Z) */ 
RawCoarseDepthMap morph_smoothing_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_smoothing_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* DYT(I,Z) = 0.5 * (E(I,Z) + D(I,Z)) */
RawCoarseDepthMap morph_dyt_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_dyt_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* TET(I,Z) = 0.5 * (O(I,Z) + C(I,Z)) */
RawCoarseDepthMap morph_tet_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_tet_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 

/* OCCO(I,Z) = 0.5 * (O(C(I,Z),Z) + C(O(I,Z),Z)) */
RawCoarseDepthMap morph_occo_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_occo_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 
