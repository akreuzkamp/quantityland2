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

namespace Quantityland2 {

namespace detail {
    template<typename T> constexpr double baseUnit = 1.0;
    template<> constexpr SI::Length baseUnit<typename base_dimensions::Length> = 0.01 * SI::units::m;
    template<> constexpr SI::Mass baseUnit<typename base_dimensions::Mass> = 0.001 * SI::units::kg;
    template<> constexpr SI::Time baseUnit<typename base_dimensions::Time> = 1.0 * SI::units::s;
    template<> constexpr SI::ElectricCurrent baseUnit<typename base_dimensions::ElectricCurrent> = 1.0 * SI::units::A;
} // namespac detail

struct CgsConfig
{
    using referenceEngine = SI;

    template<typename T> static constexpr auto baseUnit = detail::baseUnit<T>;

//     template<typename T> static constexpr double baseUnit = 1.0;
//     template<> static constexpr SI::Length baseUnit<typename base_dimensions::Length> = 0.01 * SI::units::m;
//     template<> static constexpr SI::Mass baseUnit<typename base_dimensions::Mass> = 0.001 * SI::units::kg;
//     template<> static constexpr SI::Time baseUnit<typename base_dimensions::Time> = 0.001 * SI::units::s;

    using units = SI_units_template<Engine<CgsConfig, SiDimensions>>;
    using constants = SI_constants_template<Engine<CgsConfig, SiDimensions>>;
};

using CGS = Engine<CgsConfig, SiDimensions>;

} // namespace Quantityland2

