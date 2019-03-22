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

namespace natural::detail {
    template<typename T> constexpr double baseUnit = 1.0;
    template<> constexpr SI::Mass baseUnit<typename base_dimensions::Mass> = 1.78e-36 * SI::units::kg;

    template<typename BaseDimension> constexpr int massDimension_v = 0;
    template<> constexpr int massDimension_v<base_dimensions::Length> = -1;
    template<> constexpr int massDimension_v<base_dimensions::Mass> = +1;
    template<> constexpr int massDimension_v<base_dimensions::Time> = -1;
    template<> constexpr int massDimension_v<base_dimensions::Temperature> = +1;

    template<typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
    auto massDimension(Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...>) -> Quantity<Engine, DimensionComponent<base_dimensions::Mass, ((massDimension_v<BaseDimensionPack> * exponentPack) + ...)>>;

    template<typename T>
    using MassDimension_t = decltype(massDimension(std::declval<T>()));

    template<typename MassQuantity, typename SIQuantity>
    constexpr bool canConvertToSIDimensions_v = Quantityland2::detail::hasSameDimension_v<MassQuantity, MassDimension_t<SIQuantity>>;

    template<typename ...T> constexpr double toSI_conversionFactor_v = 1.0;
    template<> constexpr double toSI_conversionFactor_v<base_dimensions::Mass> = 1.782662e-36; // 1eV / kg
    template<> constexpr double toSI_conversionFactor_v<base_dimensions::Length> = 1.97327e-7; // 1eV^-1 / m
    template<> constexpr double toSI_conversionFactor_v<base_dimensions::Time> = 6.582119e-16; // 1eV^-1 / s
    template<> constexpr double toSI_conversionFactor_v<base_dimensions::Temperature> = 1.1604505e4; // 1eV / K
//     template<> constexpr double toSI_conversionFactor_v<base_dimensions::ElectricCurrent> = 6.58e-16;


    template<typename ...Args> struct toSI_conversionFactor {};
    template<typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
    struct toSI_conversionFactor<Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...>>
    {
        static constexpr double value = (pow<exponentPack>(toSI_conversionFactor_v<BaseDimensionPack>) * ...);
    };
    template<typename Engine, typename ...DimensionPack>
    constexpr double toSI_conversionFactor_v<Quantity<Engine, DimensionPack...>> = toSI_conversionFactor<Quantity<Engine, DimensionPack...>>::value;


} // namespac detail

template<typename SoU>
struct MassDimensions
{
    using Length = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, -1>>;
    using Mass = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Time = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, -1>>;
    using ElectricCurrent = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Temperature = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;

    using Area = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, -2>>;
    using Volume = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, -3>>;
    using Velocity = double;
    using Momentum = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Acceleration = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Force = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 2>>;
    using Energy = Quantity<Engine<SoU, MassDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;
};

template<typename Engine>
struct NaturalUnits_units_template {
    static constexpr typename Engine::Mass eV = Engine::Mass::fromNumericalValue(1.0);
};

template<typename Engine>
struct NaturalUnits_constants_template {
    using units = NaturalUnits_units_template<Engine>;
    static constexpr double c = 1.0;
    static constexpr double hbar = 1.0;
    static constexpr double kB = 1.0;
    static constexpr double ke = 1.0;
//     static constexpr Quantity<Engine, DimensionComponent<base_dimensions::Length, 3>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, -2>> G = 6.67408e-11 * (units::m3 / units::kg / units::s / units::s);
};

struct NaturalConfig
{
    using referenceEngine = SI; // TODO: Remove
    template<typename T> static constexpr auto baseUnit = natural::detail::baseUnit<T>;

//     template<typename T> static constexpr double baseUnit = 1.0;
//     template<> static constexpr SI::Length baseUnit<typename base_dimensions::Length> = 0.01 * SI::units::m;
//     template<> static constexpr SI::Mass baseUnit<typename base_dimensions::Mass> = 0.001 * SI::units::kg;
//     template<> static constexpr SI::Time baseUnit<typename base_dimensions::Time> = 0.001 * SI::units::s;

    using units = NaturalUnits_units_template<Engine<NaturalConfig, MassDimensions>>;
    using constants = NaturalUnits_constants_template<Engine<NaturalConfig, MassDimensions>>;
};

using NaturalUnits = Engine<NaturalConfig, MassDimensions>;

template<typename TargetQuantity, int massDimension>
constexpr auto convertToSI(Quantity<NaturalUnits, DimensionComponent<base_dimensions::Mass, massDimension>> value)
    -> TargetQuantity
{
    static_assert(natural::detail::canConvertToSIDimensions_v<Quantity<NaturalUnits, DimensionComponent<base_dimensions::Mass, massDimension>>, TargetQuantity>,
                  "Can't convert to SI: Mass dimension of source and SI dimension of target do not match.");

    return TargetQuantity::fromNumericalValue( value.numericalValue() * natural::detail::toSI_conversionFactor_v<TargetQuantity> );
}

} // namespace Quantityland2

