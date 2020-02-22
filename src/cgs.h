/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright 2019  Anton Kreuzkamp <devel at akreuzkamp dot de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with this library; see the file License.txt.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#pragma once

#include "quantityland2.h"
#include "si.h"
#include <tuple>

namespace Quantityland2 {

struct CGSEngine
{
    using NumberType = double;
    using SystemOfDimensions = SiDimensions<CGSEngine>;
    using referenceEngine = SiEngine;

    template<typename T> static constexpr double baseUnit = 1.0;

    static constexpr std::tuple baseUnits = {
        0.001 * SI::units::kg,
        0.01 * SI::units::m,
        1.0 * SI::units::s,
        1.0 * SI::units::A,
        1.0 * SI::units::K,
        1.0 * SI::units::mol,
        1.0 * SI::units::cd
    };

    using units = SI_units_template<CGSEngine>;
    using constants = SI_constants_template<CGSEngine>;
};

using CGS = SystemOfQuantities<CGSEngine>;

} // namespace Quantityland2

