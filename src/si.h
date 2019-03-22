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

namespace Quantityland2 {

namespace detail {
    template<typename T> constexpr auto unitString = "";
    template<> constexpr auto unitString<typename base_dimensions::Length> = "m";
    template<> constexpr auto unitString<typename base_dimensions::Mass> = "kg";
    template<> constexpr auto unitString<typename base_dimensions::Time> = "s";
    template<> constexpr auto unitString<typename base_dimensions::ElectricCurrent> = "A";
    template<> constexpr auto unitString<typename base_dimensions::Temperature> = "K";
    template<> constexpr auto unitString<typename base_dimensions::AmountOfSubstance> = "mol";
    template<> constexpr auto unitString<typename base_dimensions::Luminosity> = "cd";

    template<typename Engine>
    constexpr double meter_v = 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::Length>);
    template<typename Engine>
    constexpr double kilogram_v = 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::Mass>);
    template<typename Engine>
    constexpr double second_v = 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::Time>);
    template<typename Engine>
    constexpr double ampere_v = 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::ElectricCurrent>);
    template<typename Engine>
    constexpr double kelvin_v = 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::Temperature>);
    template<typename Engine>
    constexpr double mol_v = 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::AmountOfSubstance>);
    template<typename Engine>
    constexpr double candela_v= 1.0 / toNumericalValue(Engine::template baseUnit<typename base_dimensions::Luminosity>);
}

template<typename Engine>
struct SI_units_template {
    static constexpr typename Engine::Mass kg = Engine::Mass::fromNumericalValue(1.0 * detail::kilogram_v<Engine>);
    static constexpr typename Engine::Mass g = Engine::Mass::fromNumericalValue(0.001 * detail::kilogram_v<Engine>);
    static constexpr typename Engine::Mass mg = Engine::Mass::fromNumericalValue(0.000'001 * detail::kilogram_v<Engine>);
    static constexpr typename Engine::Mass ug = Engine::Mass::fromNumericalValue(0.000'000'001 * detail::kilogram_v<Engine>);
    static constexpr typename Engine::Mass ng = Engine::Mass::fromNumericalValue(0.000'000'000'001 * detail::kilogram_v<Engine>);

    static constexpr typename Engine::Length fm = Engine::Length::fromNumericalValue(0.000'000'000'000'001 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length pm = Engine::Length::fromNumericalValue(0.000'000'000'001 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length nm = Engine::Length::fromNumericalValue(0.000'000'001 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length um = Engine::Length::fromNumericalValue(0.000'001 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length mm = Engine::Length::fromNumericalValue(0.001 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length cm = Engine::Length::fromNumericalValue(0.01 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length m = Engine::Length::fromNumericalValue(1.0 * detail::meter_v<Engine>);
    static constexpr typename Engine::Length km = Engine::Length::fromNumericalValue(1000 * detail::meter_v<Engine>);

    static constexpr typename Engine::Time fs = Engine::Time::fromNumericalValue(0.000'000'000'000'001 * detail::second_v<Engine>);
    static constexpr typename Engine::Time ps = Engine::Time::fromNumericalValue(0.000'000'000'001 * detail::second_v<Engine>);
    static constexpr typename Engine::Time ns = Engine::Time::fromNumericalValue(0.000'000'001 * detail::second_v<Engine>);
    static constexpr typename Engine::Time us = Engine::Time::fromNumericalValue(0.000'001 * detail::second_v<Engine>);
    static constexpr typename Engine::Time ms = Engine::Time::fromNumericalValue(0.001 * detail::second_v<Engine>);
    static constexpr typename Engine::Time s = Engine::Time::fromNumericalValue(1.0 * detail::second_v<Engine>);
    static constexpr typename Engine::Time min = Engine::Time::fromNumericalValue(60 * detail::second_v<Engine>);
    static constexpr typename Engine::Time h = Engine::Time::fromNumericalValue(3600 * detail::second_v<Engine>);
    static constexpr typename Engine::Time d = Engine::Time::fromNumericalValue(24 * 3600 * detail::second_v<Engine>);
    static constexpr typename Engine::Time yr = Engine::Time::fromNumericalValue(365.25 * 24 * 3600 * detail::second_v<Engine>);

    static constexpr typename Engine::ElectricCurrent fA = Engine::ElectricCurrent::fromNumericalValue(0.000'000'000'000'001 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent pA = Engine::ElectricCurrent::fromNumericalValue(0.000'000'000'001 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent nA = Engine::ElectricCurrent::fromNumericalValue(0.000'000'001 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent uA = Engine::ElectricCurrent::fromNumericalValue(0.000'001 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent mA = Engine::ElectricCurrent::fromNumericalValue(0.001 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent A = Engine::ElectricCurrent::fromNumericalValue(1.0 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent kA = Engine::ElectricCurrent::fromNumericalValue(1000 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent MA = Engine::ElectricCurrent::fromNumericalValue(1000'000 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent GA = Engine::ElectricCurrent::fromNumericalValue(1000'000'000 * detail::ampere_v<Engine>);
    static constexpr typename Engine::ElectricCurrent TA = Engine::ElectricCurrent::fromNumericalValue(1000'000'000'000 * detail::ampere_v<Engine>);

    static constexpr typename Engine::Temperature K = Engine::Temperature::fromNumericalValue(1.0 * detail::kelvin_v<Engine>);
//         static constexpr typename Engine::Temperature DegC = Engine::Temperature::fromNumericalValue(273.15 + 1.0);
//         static constexpr typename Engine::Temperature DegF = Engine::Temperature::fromNumericalValue( (459.67 + 1.0) * (5/9) );

    static constexpr typename Engine::AmountOfSubstance mol = Engine::AmountOfSubstance::fromNumericalValue(1.0 * detail::mol_v<Engine>);

    static constexpr typename Engine::Luminosity cd = Engine::Luminosity::fromNumericalValue(1.0 * detail::candela_v<Engine>);

    static constexpr typename Engine::Area m2 = Engine::Area::fromNumericalValue(1.0 * detail::meter_v<Engine> * detail::meter_v<Engine>);
    static constexpr typename Engine::Volume m3 = Engine::Volume::fromNumericalValue(1.0 * detail::meter_v<Engine> * detail::meter_v<Engine> * detail::meter_v<Engine>);

    static constexpr typename Engine::Velocity kmh = Engine::Velocity::fromNumericalValue(1.0 / 3.6 * detail::meter_v<Engine> / detail::second_v<Engine>);

    static constexpr typename Engine::Force N = Engine::Force::fromNumericalValue(1.0 * detail::kilogram_v<Engine> * detail::meter_v<Engine> / (detail::second_v<Engine> * detail::second_v<Engine>));
    static constexpr typename Engine::Energy J = Engine::Energy::fromNumericalValue(1.0 * detail::kilogram_v<Engine> * detail::meter_v<Engine> * detail::meter_v<Engine> / (detail::second_v<Engine> * detail::second_v<Engine>));
    static constexpr typename Engine::Energy eV = 1.6021766208e-19 * J;

//     static constexpr typename Engine::Capacity F = 1.0 * A * A * s * s * s * s / (kg * m * m); // FIXME
    static constexpr typename Engine::Capacity F = Engine::Capacity::fromNumericalValue(1.0 * detail::ampere_v<Engine> * detail::ampere_v<Engine> * detail::second_v<Engine> * detail::second_v<Engine> * detail::second_v<Engine> * detail::second_v<Engine> / (detail::kilogram_v<Engine> * detail::meter_v<Engine> * detail::meter_v<Engine>));
    static constexpr typename Engine::Inductance H = Engine::Inductance::fromNumericalValue(1.0 * detail::kilogram_v<Engine> * detail::meter_v<Engine> * detail::meter_v<Engine> / (detail::second_v<Engine> * detail::second_v<Engine> * detail::ampere_v<Engine> * detail::ampere_v<Engine>));
};

template<typename Engine>
struct SI_constants_template {
    using units = SI_units_template<Engine>;
    static constexpr Quantity<Engine, DimensionComponent<base_dimensions::Length, 3>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, -2>> G = 6.67408e-11 * (units::m3 / units::kg / units::s / units::s);

    static constexpr Quantity<Engine, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -1>> c0 = 299'792'458 * (units::m / units::s);
    static constexpr auto hbar = 1.054'571'800e-34 * (units::J * units::s);
    static constexpr auto kB = 1.380'649e-23 * (units::J / units::K); // valid since 20th May 2019
//     static constexpr auto mu0 = 4*M_PI*1e-7 * (units::H / units::m);
//     static constexpr auto epslion0 = 8.854'187'817'620'39e-12 * (units::F / units::m);
//     static constexpr double ke = 1.0 / (4*M_PI*epsilon0);
};


template<typename SystemOfUnits>
struct SiDimensions
{
    using Length = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>>;
    using Mass = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Time = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, 1>>;
    using ElectricCurrent = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using Temperature = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Temperature, 1>>;
    using AmountOfSubstance = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::AmountOfSubstance, 1>>;
    using Luminosity = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Luminosity, 1>>;

    using Area = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>>;
    using Volume = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 3>>;
    using Velocity = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using Momentum = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using Acceleration = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Force = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Energy = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Capacity = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 4>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
    using Inductance = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;
};

struct SiConfig
{
    using referenceEngine = void;

    template<typename T> static constexpr auto unitString = detail::unitString<T>;
    template<typename T> static constexpr double baseUnit = 1.0;

//     static constexpr Quantity<SI, DimensionComponent<Length, 3>, DimensionComponent<Mass, -1>, DimensionComponent<Time, -2>> G = ;

    using units = SI_units_template<Engine<SiConfig, SiDimensions>>;
    using constants = SI_constants_template<Engine<SiConfig, SiDimensions>>;
};

using SI = Engine<SiConfig, SiDimensions>;


namespace SI_literals {
    constexpr SI::Mass operator"" _kg(long double value)            { return SI::Mass::fromNumericalValue(value); }
    constexpr SI::Mass operator"" _kg(unsigned long long value)     { return SI::Mass::fromNumericalValue(static_cast<long double>(value)); }
    constexpr SI::Mass operator"" _g(long double value)             { return SI::Mass::fromNumericalValue(0.001 * value); }
    constexpr SI::Mass operator"" _g(unsigned long long value)      { return SI::Mass::fromNumericalValue(0.001 * value); }
    constexpr SI::Mass operator"" _mg(long double value)            { return SI::Mass::fromNumericalValue(0.000'001 * value); }
    constexpr SI::Mass operator"" _mg(unsigned long long value)     { return SI::Mass::fromNumericalValue(0.000'001 * value); }
    constexpr SI::Mass operator"" _ug(long double value)            { return SI::Mass::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::Mass operator"" _ug(unsigned long long value)     { return SI::Mass::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::Mass operator"" _ng(long double value)            { return SI::Mass::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Mass operator"" _ng(unsigned long long value)     { return SI::Mass::fromNumericalValue(0.000'000'000'001 * value); }

    constexpr SI::Length operator"" _fm(long double value)          { return SI::Length::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Length operator"" _fm(unsigned long long value)   { return SI::Length::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Length operator"" _pm(long double value)          { return SI::Length::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Length operator"" _pm(unsigned long long value)   { return SI::Length::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Length operator"" _nm(long double value)          { return SI::Length::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::Length operator"" _nm(unsigned long long value)   { return SI::Length::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::Length operator"" _um(long double value)          { return SI::Length::fromNumericalValue(0.000'001 * value); }
    constexpr SI::Length operator"" _um(unsigned long long value)   { return SI::Length::fromNumericalValue(0.000'001 * value); }
    constexpr SI::Length operator"" _mm(long double value)          { return SI::Length::fromNumericalValue(0.001 * value); }
    constexpr SI::Length operator"" _mm(unsigned long long value)   { return SI::Length::fromNumericalValue(0.001 * value); }
    constexpr SI::Length operator"" _cm(long double value)          { return SI::Length::fromNumericalValue(0.01 * value); }
    constexpr SI::Length operator"" _cm(unsigned long long value)   { return SI::Length::fromNumericalValue(0.01 * value); }
    constexpr SI::Length operator"" _m(long double value)           { return SI::Length::fromNumericalValue(value); }
    constexpr SI::Length operator"" _m(unsigned long long value)    { return SI::Length::fromNumericalValue(static_cast<long double>(value)); }
    constexpr SI::Length operator"" _km(long double value)          { return SI::Length::fromNumericalValue(1000 * value); }
    constexpr SI::Length operator"" _km(unsigned long long value)   { return SI::Length::fromNumericalValue(static_cast<long double>(1000 * value)); }

    constexpr SI::Time operator"" _fs(long double value)            { return SI::Time::fromNumericalValue(0.000'000'000'000'001 * value); }
    constexpr SI::Time operator"" _fs(unsigned long long value)     { return SI::Time::fromNumericalValue(0.000'000'000'000'001 * value); }
    constexpr SI::Time operator"" _ps(long double value)            { return SI::Time::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Time operator"" _ps(unsigned long long value)     { return SI::Time::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::Time operator"" _ns(long double value)            { return SI::Time::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::Time operator"" _ns(unsigned long long value)     { return SI::Time::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::Time operator"" _us(long double value)            { return SI::Time::fromNumericalValue(0.000'001 * value); }
    constexpr SI::Time operator"" _us(unsigned long long value)     { return SI::Time::fromNumericalValue(0.000'001 * value); }
    constexpr SI::Time operator"" _ms(long double value)            { return SI::Time::fromNumericalValue(0.001 * value); }
    constexpr SI::Time operator"" _ms(unsigned long long value)     { return SI::Time::fromNumericalValue(0.001 * value); }
    constexpr SI::Time operator"" _s(long double value)             { return SI::Time::fromNumericalValue(value); }
    constexpr SI::Time operator"" _s(unsigned long long value)      { return SI::Time::fromNumericalValue(static_cast<long double>(value)); }
    constexpr SI::Time operator"" _min(long double value)           { return SI::Time::fromNumericalValue(60 * value); }
    constexpr SI::Time operator"" _min(unsigned long long value)    { return SI::Time::fromNumericalValue(static_cast<long double>(60 * value)); }
    constexpr SI::Time operator"" _h(long double value)             { return SI::Time::fromNumericalValue(3600 * value); }
    constexpr SI::Time operator"" _h(unsigned long long value)      { return SI::Time::fromNumericalValue(static_cast<long double>(3600 * value)); }
    constexpr SI::Time operator"" _d(long double value)             { return SI::Time::fromNumericalValue(24 * 3600 * value); }
    constexpr SI::Time operator"" _d(unsigned long long value)      { return SI::Time::fromNumericalValue(static_cast<long double>(24 * 3600 * value)); }
    constexpr SI::Time operator"" _yr(long double value)            { return SI::Time::fromNumericalValue(365.25 * 24 * 3600 * value); }
    constexpr SI::Time operator"" _yr(unsigned long long value)     { return SI::Time::fromNumericalValue(static_cast<long double>(365.25 * 24 * 3600 * value)); }

    constexpr SI::ElectricCurrent operator"" _fA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(0.000'000'000'000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _fA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(0.000'000'000'000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _pA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _pA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(0.000'000'000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _nA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _nA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(0.000'000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _uA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(0.000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _uA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(0.000'001 * value); }
    constexpr SI::ElectricCurrent operator"" _mA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(0.001 * value); }
    constexpr SI::ElectricCurrent operator"" _mA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(0.001 * value); }
    constexpr SI::ElectricCurrent operator"" _A(long double value)         { return SI::ElectricCurrent::fromNumericalValue(value); }
    constexpr SI::ElectricCurrent operator"" _A(unsigned long long value)  { return SI::ElectricCurrent::fromNumericalValue(static_cast<long double>(value)); }
    constexpr SI::ElectricCurrent operator"" _kA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(1000 * value); }
    constexpr SI::ElectricCurrent operator"" _kA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(static_cast<long double>(1000 * value)); }
    constexpr SI::ElectricCurrent operator"" _MA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(1000'000 * value); }
    constexpr SI::ElectricCurrent operator"" _MA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(static_cast<long double>(1000'000 * value)); }
    constexpr SI::ElectricCurrent operator"" _GA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(1000'000'000 * value); }
    constexpr SI::ElectricCurrent operator"" _GA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(static_cast<long double>(1000'000'000 * value)); }
    constexpr SI::ElectricCurrent operator"" _TA(long double value)        { return SI::ElectricCurrent::fromNumericalValue(1000'000'000'000 * value); }
    constexpr SI::ElectricCurrent operator"" _TA(unsigned long long value) { return SI::ElectricCurrent::fromNumericalValue(static_cast<long double>(1000'000'000'000 * value)); }

    constexpr SI::Temperature operator"" _K(long double value)           { return SI::Temperature::fromNumericalValue(value); }
    constexpr SI::Temperature operator"" _K(unsigned long long value)    { return SI::Temperature::fromNumericalValue(static_cast<long double>(value)); }
    constexpr SI::Temperature operator"" _DegC(long double value)        { return SI::Temperature::fromNumericalValue(273.15 + value); }
    constexpr SI::Temperature operator"" _DegC(unsigned long long value) { return SI::Temperature::fromNumericalValue(273.15 + value); }
    constexpr SI::Temperature operator"" _DegF(long double value)        { return SI::Temperature::fromNumericalValue( (459.67 + value) * (5/9) ); }
    constexpr SI::Temperature operator"" _DegF(unsigned long long value) { return SI::Temperature::fromNumericalValue( (459.67 + value) * (5/9) ); }

    constexpr SI::AmountOfSubstance operator"" _mol(long double value)        { return SI::AmountOfSubstance::fromNumericalValue(value); }
    constexpr SI::AmountOfSubstance operator"" _mol(unsigned long long value) { return SI::AmountOfSubstance::fromNumericalValue(static_cast<long double>(value)); }

    constexpr SI::Luminosity operator"" _cd(long double value)        { return SI::Luminosity::fromNumericalValue(value); }
    constexpr SI::Luminosity operator"" _cd(unsigned long long value) { return SI::Luminosity::fromNumericalValue(static_cast<long double>(value)); }

    constexpr SI::Area operator"" _m2(long double value)   { return SI::Area::fromNumericalValue(value); }
    constexpr SI::Area operator"" _m2(unsigned long long value)   { return SI::Area::fromNumericalValue(value); }
    constexpr SI::Volume operator"" _m3(long double value)   { return SI::Volume::fromNumericalValue(value); }
    constexpr SI::Volume operator"" _m3(unsigned long long value)   { return SI::Volume::fromNumericalValue(value); }

    constexpr SI::Velocity operator"" _kmh(long double value)   { return SI::Velocity::fromNumericalValue(value / 3.6); }
    constexpr SI::Velocity operator"" _kmh(unsigned long long value)   { return SI::Velocity::fromNumericalValue(value / 3.6); }

    constexpr SI::Force operator"" _N(long double value)   { return SI::Force::fromNumericalValue(value); }
    constexpr SI::Force operator"" _N(unsigned long long value)   { return SI::Force::fromNumericalValue(value); }
    constexpr SI::Energy operator"" _J(long double value)   { return SI::Energy::fromNumericalValue(value); }
    constexpr SI::Energy operator"" _J(unsigned long long value)   { return SI::Energy::fromNumericalValue(value); }
}

} // namespace Quantityland2

