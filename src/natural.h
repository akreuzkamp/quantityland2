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

namespace natural::detail {
    template<typename Engine, Ratio mass, Ratio length, Ratio time, Ratio electriccurrent, Ratio temperature, Ratio amountofsubstance, Ratio luminousintensity>
    constexpr Ratio massDimension(Quantity<Engine, mass, length, time, electriccurrent, temperature, amountofsubstance, luminousintensity>)
    {
        return mass - length - time + temperature;
    } //FIXME add mass dimension of electriccurrent, amountofsubstance, luminousintensity

    template<typename Engine, Ratio m, typename SIQuantity>
    constexpr bool hasSameDimension(Quantity<Engine, m>, SIQuantity siQuantity)
    {
        return m == massDimension(siQuantity);
    }

    template<typename MassQuantity, typename SIQuantity>
    constexpr bool canConvertToSIDimensions_v = detail::hasSameDimension(MassQuantity::fromNumericalValue(0.0), SIQuantity::fromNumericalValue(0.0));

    // TODO: Find out higher precision conversion factors
    template<typename ...T> constexpr double toSI_conversionFactor_v = 1.0;

    template<typename Engine, Ratio mass, Ratio length, Ratio time, Ratio electriccurrent, Ratio temperature, Ratio amountofsubstance, Ratio luminousintensity>
    constexpr double toSI_conversionFactor(Quantity<Engine, mass, length, time, electriccurrent, temperature, amountofsubstance, luminousintensity>)
    {
        constexpr double massFactor = 1.782662e-36; // 1eV / kg
        constexpr double lengthFactor = 1.97327e-7; // 1eV^-1 / m
        constexpr double timeFactor = 6.582119e-16; // 1eV^-1 / s
        constexpr double temperatureFactor = 1.1604505e4; // 1eV / K
        constexpr double electriccurrentFactor = 1.0; // TODO
        constexpr double amountofsubstanceFactor = 1.0; // TODO
        constexpr double luminousintensityFactor = 1.0; // TODO

        return (pow<mass>(massFactor)
              * pow<length>(lengthFactor)
              * pow<time>(timeFactor)
              * pow<electriccurrent>(electriccurrentFactor)
              * pow<temperature>(temperatureFactor)
              * pow<amountofsubstance>(amountofsubstanceFactor)
              * pow<luminousintensity>(luminousintensityFactor));
    };

    /**
     * For a given Quantity, returns the conversion factor that needs to be multiplicated in
     * order to convert from natural units to SI units:
     *
     * That is: Let NaturalUnits::Mass m = 1.5 eV
     *  => double m_rep == 1.5
     *  => double m_converted_rep = toSI_conversionFactor_v * 1.5
     *  => SI::Mass m_converted == 1.5 eV.
     *
     * Note: This only gives the conversion factor to SI, not to SI-derived system of units. To do
     *       that, you'll need a second conversion.
     */
    template<typename Engine, Ratio ...DimensionPack>
    constexpr double toSI_conversionFactor_v<Quantity<Engine, DimensionPack...>> = toSI_conversionFactor(Quantity<Engine, DimensionPack...>::fromNumericalValue(0.0));


} // namespac detail

template<typename Engn>
struct MassDimensions
{
    using Length = Quantity<Engn, -1>;
    using Mass = Quantity<Engn, 1>;
    using Time = Quantity<Engn, -1>;
    using ElectricCurrent = Quantity<Engn, 1>;
    using Temperature = Quantity<Engn, 1>;

    using Area = Quantity<Engn, -2>;
    using Volume = Quantity<Engn, -3>;
    using Velocity = typename Engn::NumberType;
    using Momentum = Quantity<Engn, 1>;
    using Acceleration = Quantity<Engn, 1>;
    using Force = Quantity<Engn, 2>;
    using Energy = Quantity<Engn, 1>;
};

template<typename Engine>
struct NaturalUnits_units_template {
    using Q = typename Engine::SystemOfDimensions;
    static constexpr typename Q::Mass eV = Q::Mass::fromNumericalValue(1.0);
};

template<typename Engine>
struct NaturalUnits_constants_template {
    using units = NaturalUnits_units_template<Engine>;
    static constexpr double c = 1.0;
    static constexpr double hbar = 1.0;
    static constexpr double kB = 1.0;
    static constexpr double ke = 1.0;
};

struct NaturalEngine
{
    using NumberType = double;
    using SystemOfDimensions = MassDimensions<NaturalEngine>;
    using referenceEngine = SiEngine; // TODO: Remove
    static constexpr std::array<const char *, 1> unitStrings = { "eV" };

    /**
     * Converts a quantity from natural units to \p TargetQuantity.
     *
     * \p TargetQuantity can have any dimension whose mass dimension equals the mass dimension of
     * \p inputValue (e.g. NaturalUnits::Length can be converted to SI::Time, but not to SI::Mass
     * or SI::Area). The necessary constants are added automatically.
     *
     * \p TargetQuantity can have any system of units which uses SI dimensions and allows conversion
     * from SI units. Conversion factors will be inserted as necessary.
     *
     * \p TargetQuantity is an explicit template parameter, \p massDimension can be deduced from
     * \p inputValue.
     */
    template<typename TargetQuantity, Ratio massDimension>
    static constexpr TargetQuantity toSiDimensions(Quantity<NaturalEngine, massDimension> inputValue)
    {
        static_assert(natural::detail::canConvertToSIDimensions_v<Quantity<NaturalEngine, massDimension>, TargetQuantity>,
                      "Can't convert to SI: Mass dimension of source and SI dimension of target do not match.");

        using IntermediateSIQuantity = detail::changeEngine_t<SiEngine, TargetQuantity>;
        IntermediateSIQuantity tmp = IntermediateSIQuantity::fromNumericalValue( inputValue.numericalValue() * natural::detail::toSI_conversionFactor_v<IntermediateSIQuantity> );

        return static_cast<TargetQuantity>(tmp); // Does the implicit conversion from SI units into the actual target system of units
    }

    /**
     * Converts a quantity from any SI-based SourceQuantity to natural units.
     *
     * \p inputValue can have any dimension. The mass dimension will be automatically deduced.
     *
     * \p SourceEngine can have any system of units which uses SI dimensions and allows conversion
     * to SI units. Conversion factors will be inserted as necessary.
     */
    template<typename SourceEngine, Ratio ...DimensionPack>
    static constexpr auto fromSiDimensions(Quantity<SourceEngine, DimensionPack...> inputValue)
    -> Quantity<NaturalEngine, natural::detail::massDimension(Quantity<SourceEngine, DimensionPack...>::fromNumericalValue(0.0))>
    {
        Quantity<SiEngine, DimensionPack...> tmp = inputValue;
        using Ret = Quantity<NaturalEngine, natural::detail::massDimension(Quantity<SourceEngine, DimensionPack...>::fromNumericalValue(0.0))>;

        return Ret::fromNumericalValue(
            tmp.numericalValue() / natural::detail::toSI_conversionFactor_v<Quantity<SourceEngine, DimensionPack...>>
        );
    }

    using units = NaturalUnits_units_template<NaturalEngine>;
    using constants = NaturalUnits_constants_template<NaturalEngine>;
};

using NaturalUnits = SystemOfQuantities<NaturalEngine>;

} // namespace Quantityland2

