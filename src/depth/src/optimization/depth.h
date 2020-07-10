#pragma once

#include <libv/lma/lma.hpp>

#include "../types.h"

namespace ttt
{
    template<>
    struct Name<Depth>{ static std::string name(){ return "Depth"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Depth
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<Depth>{ enum{ value = 1 }; };

    void apply_increment(Depth& depth, const double delta[Size<Depth>::value], const Adl&);
    void apply_small_increment(Depth& depth, const double h, const v::core::numeric_tag<0>&, const Adl&);
} // namespace lma
