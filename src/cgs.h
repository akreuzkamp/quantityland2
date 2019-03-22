/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright 2019  Anton Kreuzkamp <anton.kreuzkamp@kdab.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License or (at your option) version 3 or any later version
 * accepted by the membership of KDE e.V. (or its successor approved
 * by the membership of KDE e.V.), which shall act as a proxy
 * defined in Section 14 of version 3 of the license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
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

