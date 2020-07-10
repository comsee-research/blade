//#include "optimization.h"
#include "depth.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Depth
////////////////////////////////////////////////////////////////////////////////////////////////////
	void apply_increment(Depth& depth, const double delta[Size<Depth>::value], const Adl&)
	{
		depth.z += delta[0];
	}

	void apply_small_increment(Depth& depth, const double h, const v::core::numeric_tag<0>&, const Adl&)
	{
		depth.z += h;
	}
} // namespace lma
