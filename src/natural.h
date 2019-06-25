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
    template<typename BaseDimension> constexpr int massDimension_v = 0;
    template<> constexpr int massDimension_v<base_dimensions::Length> = -1;
    template<> constexpr int massDimension_v<base_dimensions::Mass> = +1;
    template<> constexpr int massDimension_v<base_dimensions::Time> = -1;
    template<> constexpr int massDimension_v<base_dimensions::Temperature> = +1;

    template<typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
    auto massDimension(Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...>) -> Quantity<Engine, DimensionComponent<base_dimensions::Mass, ((massDimension_v<BaseDimensionPack> * exponentPack) + ...)>>;

    /**
     * Converts SI Dimensions to Mass Dimensions.
     *
     * Note: This neither changes the system of units, nor the system of dimensions!
     *       Merely, the dimension packs are altered. The Engine is not touched at all.
     */
    template<typename T>
    using MassDimension_t = decltype(massDimension(std::declval<T>()));

    template<typename MassQuantity, typename SIQuantity>
    constexpr bool canConvertToSIDimensions_v = Quantityland2::detail::hasSameDimension_v<MassQuantity, MassDimension_t<SIQuantity>>;

    // TODO: Find out higher precision conversion factors
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
    template<typename Engine, typename ...DimensionPack>
    constexpr double toSI_conversionFactor_v<Quantity<Engine, DimensionPack...>> = toSI_conversionFactor<Quantity<Engine, DimensionPack...>>::value;


} // namespac detail

template<typename Engn>
struct MassDimensions
{
    using Length = Quantity<Engn, DimensionComponent<base_dimensions::Mass, -1>>;
    using Mass = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 1>>;
    using Time = Quantity<Engn, DimensionComponent<base_dimensions::Mass, -1>>;
    using ElectricCurrent = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 1>>;
    using Temperature = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 1>>;

    using Area = Quantity<Engn, DimensionComponent<base_dimensions::Mass, -2>>;
    using Volume = Quantity<Engn, DimensionComponent<base_dimensions::Mass, -3>>;
    using Velocity = double;
    using Momentum = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 1>>;
    using Acceleration = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 1>>;
    using Force = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 2>>;
    using Energy = Quantity<Engn, DimensionComponent<base_dimensions::Mass, 1>>;
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

struct NaturalUnits : public MassDimensions<NaturalUnits>
{
    using referenceEngine = SI; // TODO: Remove
    template<typename T> static constexpr double baseUnit = 1.0;
    template<typename T> static constexpr auto unitString = "";

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
    template<typename TargetQuantity, int massDimension>
    static constexpr auto toSiDimensions(Quantity<NaturalUnits, DimensionComponent<base_dimensions::Mass, massDimension>> inputValue)
    -> TargetQuantity
    {
        static_assert(natural::detail::canConvertToSIDimensions_v<Quantity<NaturalUnits, DimensionComponent<base_dimensions::Mass, massDimension>>, TargetQuantity>,
                      "Can't convert to SI: Mass dimension of source and SI dimension of target do not match.");

        using IntermediateSIQuantity = detail::changeEngine_t<SI, TargetQuantity>;
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
    template<typename SourceEngine, typename ...DimensionPack>
    static constexpr auto fromSiDimensions(Quantity<SourceEngine, DimensionPack...> inputValue)
    -> natural::detail::MassDimension_t<Quantity<NaturalUnits, DimensionPack...>>
    {
        Quantity<SI, DimensionPack...> tmp = inputValue;

        return natural::detail::MassDimension_t<Quantity<NaturalUnits, DimensionPack...>>::fromNumericalValue(
            tmp.numericalValue() / natural::detail::toSI_conversionFactor_v<Quantity<SourceEngine, DimensionPack...>>
        );
    }

    using units = NaturalUnits_units_template<NaturalUnits>;
    using constants = NaturalUnits_constants_template<NaturalUnits>;
};
template<> constexpr SI::Mass NaturalUnits::baseUnit<typename base_dimensions::Mass> = 1.782662e-36 * SI::units::kg;
template<> constexpr auto NaturalUnits::unitString<typename base_dimensions::Mass> = "eV";

} // namespace Quantityland2

