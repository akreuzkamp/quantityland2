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
    // Base units
    static constexpr typename Engine::Mass kg = Engine::Mass::fromNumericalValue(1.0 * detail::kilogram_v<Engine>);
    static constexpr typename Engine::Mass g = 1.0e-3 * kg;
    static constexpr typename Engine::Mass mg = 1.0e-6 * kg;
    static constexpr typename Engine::Mass ug = 1.0e-9 * kg;
    static constexpr typename Engine::Mass ng = 1.0e-12 * kg;
    static constexpr typename Engine::Mass t = 1.0e+3 * kg;
    static constexpr typename Engine::Mass kt = 1.0e+6 * kg;
    static constexpr typename Engine::Mass Mt = 1.0e+9 * kg;
    static constexpr typename Engine::Mass Gt = 1.0e+12 * kg;


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

    static constexpr typename Engine::Area m2 = m * m;
    static constexpr typename Engine::Area cm2 = 1.0e-4 * m2;
    static constexpr typename Engine::Area mm2 = 1.0e-6 * m2;
    static constexpr typename Engine::Area a = 100 * m2;
    static constexpr typename Engine::Area daa = 1000 * m2;
    static constexpr typename Engine::Area ha = 10'000 * m2;
    static constexpr typename Engine::Area km2 = 1'000'000 * m2;

    static constexpr typename Engine::Volume m3 = m * m * m;
    static constexpr typename Engine::Volume ml = 1e-6 * m3;
    static constexpr typename Engine::Volume cl = 1e-5 * m3;
    static constexpr typename Engine::Volume l = 1e-3 * m3;

    static constexpr typename Engine::Velocity kmh = Engine::Velocity::fromNumericalValue(1.0 / 3.6 * detail::meter_v<Engine> / detail::second_v<Engine>);


    static constexpr double rad = 1.0; // radian
    static constexpr double sr = 1.0; // steradian

    static constexpr typename Engine::Capacitance F = A*A * s*s*s*s / kg / m / m; // farad
    static constexpr typename Engine::Capacitance mF = 1.0e-3 * F;
    static constexpr typename Engine::Capacitance uF = 1.0e-6 * F;
    static constexpr typename Engine::Capacitance nF = 1.0e-9 * F;
    static constexpr typename Engine::Capacitance pF = 1.0e-12 * F;
    static constexpr typename Engine::Capacitance fF = 1.0e-15 * F;

    static constexpr typename Engine::ElectricCharge C = A * s; // coulomb
    static constexpr typename Engine::ElectricCharge mC = 1.0e-3  * C;
    static constexpr typename Engine::ElectricCharge uC = 1.0e-6  * C;
    static constexpr typename Engine::ElectricCharge nC = 1.0e-9  * C;
    static constexpr typename Engine::ElectricCharge pC = 1.0e-12 * C;
    static constexpr typename Engine::ElectricCharge fC = 1.0e-15 * C;
    static constexpr typename Engine::ElectricCharge kC = 1.0e+3  * C;
    static constexpr typename Engine::ElectricCharge MC = 1.0e+6  * C;
    static constexpr typename Engine::ElectricCharge GC = 1.0e+9  * C;
    static constexpr typename Engine::ElectricCharge TC = 1.0e+12 * C;

    static constexpr typename Engine::CatalyticActivity kat = mol / s; // katal
    static constexpr typename Engine::DoseEquivalent Sv = m * m / s / s; // sievert
    static constexpr typename Engine::DoseEquivalent mSv = 1.0e-3  * Sv;
    static constexpr typename Engine::DoseEquivalent uSv = 1.0e-6  * Sv;
    static constexpr typename Engine::DoseEquivalent nSv = 1.0e-9  * Sv;
    static constexpr typename Engine::DoseEquivalent pSv = 1.0e-12 * Sv;
    static constexpr typename Engine::DoseEquivalent fSv = 1.0e-15 * Sv;

    static constexpr typename Engine::ElectricalConductance S = A * A * s * s * s / kg / m / m; // siemens
    static constexpr typename Engine::ElectricalConductance mS = 1.0e-3  * S;
    static constexpr typename Engine::ElectricalConductance uS = 1.0e-6  * S;
    static constexpr typename Engine::ElectricalConductance nS = 1.0e-9  * S;
    static constexpr typename Engine::ElectricalConductance pS = 1.0e-12 * S;
    static constexpr typename Engine::ElectricalConductance fS = 1.0e-15 * S;
    static constexpr typename Engine::ElectricalConductance kS = 1.0e+3  * S;
    static constexpr typename Engine::ElectricalConductance MS = 1.0e+6  * S;
    static constexpr typename Engine::ElectricalConductance GS = 1.0e+9  * S;
    static constexpr typename Engine::ElectricalConductance TS = 1.0e+12 * S;

    static constexpr typename Engine::ElectricPotential V = kg * m * m / A / s / s /s; // volt
    static constexpr typename Engine::ElectricPotential mV = 1.0e-3  * V;
    static constexpr typename Engine::ElectricPotential uV = 1.0e-6  * V;
    static constexpr typename Engine::ElectricPotential nV = 1.0e-9  * V;
    static constexpr typename Engine::ElectricPotential pV = 1.0e-12 * V;
    static constexpr typename Engine::ElectricPotential fV = 1.0e-15 * V;
    static constexpr typename Engine::ElectricPotential kV = 1.0e+3  * V;
    static constexpr typename Engine::ElectricPotential MV = 1.0e+6  * V;
    static constexpr typename Engine::ElectricPotential GV = 1.0e+9  * V;
    static constexpr typename Engine::ElectricPotential TV = 1.0e+12 * V;

    static constexpr typename Engine::ElectricalResistance Ω = kg * m * m / A / A / s / s / s; // ohm
    static constexpr typename Engine::ElectricalResistance mΩ = 1.0e-3  * Ω;
    static constexpr typename Engine::ElectricalResistance uΩ = 1.0e-6  * Ω;
    static constexpr typename Engine::ElectricalResistance nΩ = 1.0e-9  * Ω;
    static constexpr typename Engine::ElectricalResistance pΩ = 1.0e-12 * Ω;
    static constexpr typename Engine::ElectricalResistance fΩ = 1.0e-15 * Ω;
    static constexpr typename Engine::ElectricalResistance kΩ = 1.0e+3  * Ω;
    static constexpr typename Engine::ElectricalResistance MΩ = 1.0e+6  * Ω;
    static constexpr typename Engine::ElectricalResistance GΩ = 1.0e+9  * Ω;
    static constexpr typename Engine::ElectricalResistance TΩ = 1.0e+12 * Ω;

    static constexpr typename Engine::Energy J = m * m * kg / s / s; // joule
    static constexpr typename Engine::Energy mJ = 1.0e-3  * J;
    static constexpr typename Engine::Energy uJ = 1.0e-6  * J;
    static constexpr typename Engine::Energy nJ = 1.0e-9  * J;
    static constexpr typename Engine::Energy pJ = 1.0e-12 * J;
    static constexpr typename Engine::Energy fJ = 1.0e-15 * J;
    static constexpr typename Engine::Energy kJ = 1.0e+3  * J;
    static constexpr typename Engine::Energy MJ = 1.0e+6  * J;
    static constexpr typename Engine::Energy GJ = 1.0e+9  * J;
    static constexpr typename Engine::Energy TJ = 1.0e+12 * J;

    static constexpr typename Engine::Energy eV = 1.6021766208e-19 * J; // electronvolt
    static constexpr typename Engine::Energy meV = 1.0e-3  * eV;
    static constexpr typename Engine::Energy ueV = 1.0e-6  * eV;
    static constexpr typename Engine::Energy neV = 1.0e-9  * eV;
    static constexpr typename Engine::Energy peV = 1.0e-12 * eV;
    static constexpr typename Engine::Energy feV = 1.0e-15 * eV;
    static constexpr typename Engine::Energy keV = 1.0e+3  * eV;
    static constexpr typename Engine::Energy MeV = 1.0e+6  * eV;
    static constexpr typename Engine::Energy GeV = 1.0e+9  * eV;
    static constexpr typename Engine::Energy TeV = 1.0e+12 * eV;

    static constexpr typename Engine::Energy Wh = 3'600 * J; // watt hour
    static constexpr typename Engine::Energy mWh = 1.0e-3  * Wh;
    static constexpr typename Engine::Energy uWh = 1.0e-6  * Wh;
    static constexpr typename Engine::Energy nWh = 1.0e-9  * Wh;
    static constexpr typename Engine::Energy pWh = 1.0e-12 * Wh;
    static constexpr typename Engine::Energy fWh = 1.0e-15 * Wh;
    static constexpr typename Engine::Energy kWh = 1.0e+3  * Wh;
    static constexpr typename Engine::Energy MWh = 1.0e+6  * Wh;
    static constexpr typename Engine::Energy GWh = 1.0e+9  * Wh;
    static constexpr typename Engine::Energy TWh = 1.0e+12 * Wh;

    static constexpr typename Engine::Force N = kg * m / s / s; // newton
    static constexpr typename Engine::Force mN = 1.0e-3  * N;
    static constexpr typename Engine::Force uN = 1.0e-6  * N;
    static constexpr typename Engine::Force nN = 1.0e-9  * N;
    static constexpr typename Engine::Force pN = 1.0e-12 * N;
    static constexpr typename Engine::Force fN = 1.0e-15 * N;
    static constexpr typename Engine::Force kN = 1.0e+3  * N;
    static constexpr typename Engine::Force MN = 1.0e+6  * N;
    static constexpr typename Engine::Force GN = 1.0e+9  * N;
    static constexpr typename Engine::Force TN = 1.0e+12 * N;

    static constexpr typename Engine::Frequency Hz = 1.0 / s; // hertz
    static constexpr typename Engine::Frequency mHz = 1.0e-3  * Hz;
    static constexpr typename Engine::Frequency uHz = 1.0e-6  * Hz;
    static constexpr typename Engine::Frequency nHz = 1.0e-9  * Hz;
    static constexpr typename Engine::Frequency pHz = 1.0e-12 * Hz;
    static constexpr typename Engine::Frequency fHz = 1.0e-15 * Hz;
    static constexpr typename Engine::Frequency kHz = 1.0e+3  * Hz;
    static constexpr typename Engine::Frequency MHz = 1.0e+6  * Hz;
    static constexpr typename Engine::Frequency GHz = 1.0e+9  * Hz;
    static constexpr typename Engine::Frequency THz = 1.0e+12 * Hz;

    static constexpr typename Engine::Inductance H = kg * m * m / A / A / s / s; // henry
    static constexpr typename Engine::Inductance mH = 1.0e-3  * H;
    static constexpr typename Engine::Inductance uH = 1.0e-6  * H;
    static constexpr typename Engine::Inductance nH = 1.0e-9  * H;
    static constexpr typename Engine::Inductance pH = 1.0e-12 * H;
    static constexpr typename Engine::Inductance fH = 1.0e-15 * H;
    static constexpr typename Engine::Inductance kH = 1.0e+3  * H;
    static constexpr typename Engine::Inductance MH = 1.0e+6  * H;
    static constexpr typename Engine::Inductance GH = 1.0e+9  * H;
    static constexpr typename Engine::Inductance TH = 1.0e+12 * H;

    static constexpr typename Engine::Power W = J / s; // watt
    static constexpr typename Engine::Power mW = 1.0e-3  * W;
    static constexpr typename Engine::Power uW = 1.0e-6  * W;
    static constexpr typename Engine::Power nW = 1.0e-9  * W;
    static constexpr typename Engine::Power pW = 1.0e-12 * W;
    static constexpr typename Engine::Power fW = 1.0e-15 * W;
    static constexpr typename Engine::Power kW = 1.0e+3  * W;
    static constexpr typename Engine::Power MW = 1.0e+6  * W;
    static constexpr typename Engine::Power GW = 1.0e+9  * W;
    static constexpr typename Engine::Power TW = 1.0e+12 * W;

    static constexpr typename Engine::MagneticFlux Wb = kg * m * m / A / s / s; // weber
    static constexpr typename Engine::MagneticFlux mWb = 1.0e-3  * Wb;
    static constexpr typename Engine::MagneticFlux uWb = 1.0e-6  * Wb;
    static constexpr typename Engine::MagneticFlux nWb = 1.0e-9  * Wb;
    static constexpr typename Engine::MagneticFlux pWb = 1.0e-12 * Wb;
    static constexpr typename Engine::MagneticFlux fWb = 1.0e-15 * Wb;
    static constexpr typename Engine::MagneticFlux kWb = 1.0e+3  * Wb;
    static constexpr typename Engine::MagneticFlux MWb = 1.0e+6  * Wb;
    static constexpr typename Engine::MagneticFlux GWb = 1.0e+9  * Wb;
    static constexpr typename Engine::MagneticFlux TWb = 1.0e+12 * Wb;

    static constexpr typename Engine::MagneticFluxDensity T = kg / A / s / s; // tesla
    static constexpr typename Engine::MagneticFluxDensity mT = 1.0e-3  * T;
    static constexpr typename Engine::MagneticFluxDensity uT = 1.0e-6  * T;
    static constexpr typename Engine::MagneticFluxDensity nT = 1.0e-9  * T;
    static constexpr typename Engine::MagneticFluxDensity pT = 1.0e-12 * T;
    static constexpr typename Engine::MagneticFluxDensity fT = 1.0e-15 * T;
    static constexpr typename Engine::MagneticFluxDensity kT = 1.0e+3  * T;
    static constexpr typename Engine::MagneticFluxDensity MT = 1.0e+6  * T;
    static constexpr typename Engine::MagneticFluxDensity GT = 1.0e+9  * T;
    static constexpr typename Engine::MagneticFluxDensity TT = 1.0e+12 * T;

    static constexpr typename Engine::Pressure Pa = kg / m / s / s; // pascal
    static constexpr typename Engine::Pressure mPa = 1.0e-3  * Pa;
    static constexpr typename Engine::Pressure uPa = 1.0e-6  * Pa;
    static constexpr typename Engine::Pressure nPa = 1.0e-9  * Pa;
    static constexpr typename Engine::Pressure pPa = 1.0e-12 * Pa;
    static constexpr typename Engine::Pressure fPa = 1.0e-15 * Pa;
    static constexpr typename Engine::Pressure hPa = 1.0e+2  * Pa;
    static constexpr typename Engine::Pressure kPa = 1.0e+3  * Pa;
    static constexpr typename Engine::Pressure MPa = 1.0e+6  * Pa;
    static constexpr typename Engine::Pressure GPa = 1.0e+9  * Pa;
    static constexpr typename Engine::Pressure TPa = 1.0e+12 * Pa;

    static constexpr typename Engine::Pressure bar = 1.0e+5 * Pa; // bar
    static constexpr typename Engine::Pressure mbar = 1.0e+2 * Pa;
    static constexpr typename Engine::Pressure atm = 1'013.25 * hPa; // atmosphere

    static constexpr typename Engine::Activity Bq = 1.0 / s; // becquerel
    static constexpr typename Engine::Activity mBq = 1.0e-3  * Bq;
    static constexpr typename Engine::Activity uBq = 1.0e-6  * Bq;
    static constexpr typename Engine::Activity nBq = 1.0e-9  * Bq;
    static constexpr typename Engine::Activity pBq = 1.0e-12 * Bq;
    static constexpr typename Engine::Activity fBq = 1.0e-15 * Bq;
    static constexpr typename Engine::Activity kBq = 1.0e+3  * Bq;
    static constexpr typename Engine::Activity MBq = 1.0e+6  * Bq;
    static constexpr typename Engine::Activity GBq = 1.0e+9  * Bq;
    static constexpr typename Engine::Activity TBq = 1.0e+12 * Bq;


    static constexpr typename Engine::Illuminance lx = cd * sr / m / m; // lux
    static constexpr typename Engine::LuminousFlux lm = cd * sr; // lumen
    static constexpr typename Engine::Dose Gy = m * m / s / s; // gray


    // Imperial units

    // Mass
    static constexpr typename Engine::Mass gr = 64.798'91 * mg;
    static constexpr typename Engine::Mass dr = 1.771'845'195'312'5 * g;
    static constexpr typename Engine::Mass oz = 28.349'523'125 * g;
    static constexpr typename Engine::Mass lb = 453.592'37 * g;
    static constexpr typename Engine::Mass cwt = 45.359'237 * kg;
    static constexpr typename Engine::Mass long_hundredweight = 50.802'345'44 * kg;
    static constexpr typename Engine::Mass short_ton = 907.184'74 * kg;
    static constexpr typename Engine::Mass long_ton = 1'016.046'908'8 * kg;

    // Length
    static constexpr typename Engine::Length p = 0.3527777777777778 * mm;
    static constexpr typename Engine::Length pica = 4.233333333333333 * mm;
    static constexpr typename Engine::Length in = 25.4 * mm;
    static constexpr typename Engine::Length ft = 0.3048 * m;
    static constexpr typename Engine::Length yd = 0.9144 * m;
    static constexpr typename Engine::Length mi = 1'609.344 * m;


    // Area
    static constexpr typename Engine::Area ft2 = 0.092'903'41 * m2;
    static constexpr typename Engine::Area ch2 = 404.6873 * m2;
    static constexpr typename Engine::Area acre = 4'046.873 * m2;
    static constexpr typename Engine::Area section = 2.589'998 * km2;
    static constexpr typename Engine::Area twp = 93.239'93 * km2;

    // Volume - US customary units
    static constexpr typename Engine::Volume minim = 61.611'519'921'875e-6 * l;
    static constexpr typename Engine::Volume fldr = 3.696'691'195'312'5 * ml;
    static constexpr typename Engine::Volume tsp = 4.928'921'593'75 * ml;
    static constexpr typename Engine::Volume Tbsp = 14.786'764'781'25 * ml;
    static constexpr typename Engine::Volume floz = 29.573'529'562'5 * ml;
    static constexpr typename Engine::Volume jig = 44.360'294'343'75 * ml;
    static constexpr typename Engine::Volume gi = 118.294'118'25 * ml;
    static constexpr typename Engine::Volume cp = 236.588'236'5 * ml;
    static constexpr typename Engine::Volume liquid_pint = 473.176'473 * ml;
    static constexpr typename Engine::Volume liquid_quart = 0.946'352'946 * l;
    static constexpr typename Engine::Volume liquid_gallon = 3.785'411'784 * l;
    static constexpr typename Engine::Volume liquid_barrel = 119.240'471'196 * l;
    static constexpr typename Engine::Volume oil_barrel = 158.987'294'928 * l;
    static constexpr typename Engine::Volume hogshead = 238.480'942'392 * l;

    static constexpr typename Engine::Volume dry_pint = 550.610'471'357'5 * ml;
    static constexpr typename Engine::Volume dry_quart = 1.101'220'942'715 * l;
    static constexpr typename Engine::Volume dry_gallon = 4.404'883'770'86 * l;
    static constexpr typename Engine::Volume pk = 8.809'767'541'72 * l;
    static constexpr typename Engine::Volume bu = 35.239'070'166'88 * l;
    static constexpr typename Engine::Volume dry_barrel = 115.627'123'584 * l;





    // nautical
    static constexpr typename Engine::Length ftm = 1.8288 * m;
    static constexpr typename Engine::Length cb = 219.456 * m;
    static constexpr typename Engine::Length nmi = 1'852.0 * m;
    static constexpr typename Engine::Velocity kn = 1'852.0 * kmh;

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

//     using Area = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>>;
//     using Volume = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 3>>;
//     using Velocity = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -1>>;
//     using Momentum = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
//     using Acceleration = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -2>>;
//     using Force = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
//     using Energy = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
//     using Capacity = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 4>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
//     using Inductance = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;


    using Absement  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, 1>>;
    using AbsorbedDoseRate  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Time, -3>>;
    using Acceleration  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using AngularAcceleration  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, -2>>;
    using AngularMomentum  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using AngularSpeed = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, -1>>;
    using Area  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>>;
    using AreaDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Capacitance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 4>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
    using CatalyticActivity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, -1>, DimensionComponent<base_dimensions::AmountOfSubstance, 1>>;
    using CatalyticActivityConcentration  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::Time, -1>, DimensionComponent<base_dimensions::AmountOfSubstance, 1>>;
    using ChemicalPotential  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::AmountOfSubstance, -1>>;
    using Crackle  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -5>>;
    using CurrentDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using DoseEquivalent  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Time, -2>>;
    using DynamicViscosity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using ElectricCharge  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, 1>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using ElectricChargeDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::Time, 1>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using ElectricDisplacement  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Time, 1>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using ElectricFieldStrength  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>, DimensionComponent<base_dimensions::ElectricCurrent, -1>>;
    using ElectricalConductance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 3>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
    using ElectricalConductivity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 3>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
    using ElectricPotential  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>, DimensionComponent<base_dimensions::ElectricCurrent, -1>>;
    using ElectricalResistance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;
    using ElectricalResistivity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 3>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;
    using Energy  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using EnergyDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Entropy  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::Temperature, -1>>;
    using Force  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Frequency  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, -1>>;
    using FuelEfficiency  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>>;
    using HalfLife  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, 1>>;
    using Heat  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using HeatCapacity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::Temperature, -1>>;
    using HeatFluxDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using Illuminance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Luminosity, 1>>;
    using Impedance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;
    using Impulse  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using Inductance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;
    using Irradiance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using Intensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using Jerk  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using Jounce   = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -4>>;
    using KinematicViscosity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Time, -1>>;
    using LinearDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Mass, 1>>;
    using LuminousFlux   = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Luminosity, 1>>;
    using MagneticFieldStrength  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using MagneticFlux  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::ElectricCurrent, -1>>;
    using MagneticFluxDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::ElectricCurrent, -1>>;
    using Magnetization  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::ElectricCurrent, 1>>;
    using MassDensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::Mass, 1>>;
    using MeanLifetime  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, 1>>;
    using MolarConcentration  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::AmountOfSubstance, 1>>;
    using MolarEnergy  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::AmountOfSubstance, -1>>;
    using MolarEntropy  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::Temperature, -1>,  DimensionComponent<base_dimensions::AmountOfSubstance, -1>>;
    using MolarHeatCapacity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::Temperature, -1>,  DimensionComponent<base_dimensions::AmountOfSubstance, -1>>;
    using MomentOfInertia  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>>;
    using Momentum  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using Permeability  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::ElectricCurrent, -2>>;
    using Permittivity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 4>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
    using Power  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using Pressure  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Pop  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -6>>;
    using Activity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Time, -1>>;
    using Dose  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Time, -2>>;
    using Radiance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using RadiantIntensity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>>;
    using ReactionRate  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -3>, DimensionComponent<base_dimensions::Time, -1>, DimensionComponent<base_dimensions::AmountOfSubstance, 1>>;
    using Reluctance  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -2>, DimensionComponent<base_dimensions::Mass, -1>, DimensionComponent<base_dimensions::Time, 2>, DimensionComponent<base_dimensions::ElectricCurrent, 2>>;
    using Speed  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using SpecificEnergy  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Time, -2>>;
    using SpecificHeatCapacity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Time, -2>, DimensionComponent<base_dimensions::Temperature, -1>>;
    using SpecificVolume  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 3>, DimensionComponent<base_dimensions::Mass, -1>>;
    using Spin  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using Stress  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using SurfaceTension  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using TemperatureGradient  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Temperature, 1>>;
    using ThermalConductivity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -3>, DimensionComponent<base_dimensions::Temperature, -1>>;
    using Torque  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Velocity  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Time, -1>>;
    using Volume  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 3>>;
    using VolumetricFlowRate  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 3>, DimensionComponent<base_dimensions::Time, -1>>;
    using Wavelength  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>>;
    using Wavenumber  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>>;
    using Wavevector  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>>;
    using Weight  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using Work  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, 2>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;
    using YoungsModulus  = Quantity<Engine<SystemOfUnits, SiDimensions>, DimensionComponent<base_dimensions::Length, -1>, DimensionComponent<base_dimensions::Mass, 1>, DimensionComponent<base_dimensions::Time, -2>>;

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

