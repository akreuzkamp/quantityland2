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
#include <array>


namespace Quantityland2 {

namespace detail {
    template<typename Engine>
    constexpr double kilogram_v = 1.0 / toNumericalValue(std::get<0>(Engine::baseUnits));
    template<typename Engine>
    constexpr double meter_v = 1.0 / toNumericalValue(std::get<1>(Engine::baseUnits));
    template<typename Engine>
    constexpr double second_v = 1.0 / toNumericalValue(std::get<2>(Engine::baseUnits));
    template<typename Engine>
    constexpr double ampere_v = 1.0 / toNumericalValue(std::get<3>(Engine::baseUnits));
    template<typename Engine>
    constexpr double kelvin_v = 1.0 / toNumericalValue(std::get<4>(Engine::baseUnits));
    template<typename Engine>
    constexpr double mol_v = 1.0 / toNumericalValue(std::get<5>(Engine::baseUnits));
    template<typename Engine>
    constexpr double candela_v= 1.0 / toNumericalValue(std::get<6>(Engine::baseUnits));
}

template<typename Engine>
struct SI_units_template {
    using Q = typename Engine::SystemOfDimensions;
    // Base units
    static constexpr typename Q::Mass kg = Q::Mass::fromNumericalValue(1.0 * detail::kilogram_v<Engine>);
    static constexpr typename Q::Mass g = 1.0e-3 * kg;
    static constexpr typename Q::Mass mg = 1.0e-6 * kg;
    static constexpr typename Q::Mass ug = 1.0e-9 * kg;
    static constexpr typename Q::Mass ng = 1.0e-12 * kg;
    static constexpr typename Q::Mass t = 1.0e+3 * kg;
    static constexpr typename Q::Mass kt = 1.0e+6 * kg;
    static constexpr typename Q::Mass Mt = 1.0e+9 * kg;
    static constexpr typename Q::Mass Gt = 1.0e+12 * kg;
    static constexpr typename Q::Mass u = 1.660539040e-27 * kg;
    static constexpr typename Q::Mass Da = u;


    static constexpr Quantity<Engine, 2, 0, 0, 0, 0, 0, 0> kg2 = kg * kg; // to ease writing of other units, like F = A2 * s4 / kg / m2
    static constexpr Quantity<Engine, 3, 0, 0, 0, 0, 0, 0> kg3 = kg * kg * kg;
    static constexpr Quantity<Engine, 4, 0, 0, 0, 0, 0, 0> kg4 = kg * kg * kg * kg;

    static constexpr typename Q::Length m = Q::Length::fromNumericalValue(1.0 * detail::meter_v<Engine>);
    static constexpr typename Q::Length cm = 1.0e-2  * m;
    static constexpr typename Q::Length mm = 1.0e-3  * m;
    static constexpr typename Q::Length um = 1.0e-6  * m;
    static constexpr typename Q::Length nm = 1.0e-9  * m;
    static constexpr typename Q::Length pm = 1.0e-12 * m;
    static constexpr typename Q::Length fm = 1.0e-15 * m;
    static constexpr typename Q::Length km = 1.0e+3  * m;
    static constexpr typename Q::Length au = 149'597'870'700 * m;
    static constexpr Quantity<Engine, 0, 4, 0, 0, 0, 0, 0> m4 = m * m * m * m; // to ease writing of other units, like F = A2 * s4 / kg / m2


    static constexpr typename Q::Time s = Q::Time::fromNumericalValue(1.0 * detail::second_v<Engine>);
    static constexpr typename Q::Time ms = 1.0e-3  * s;
    static constexpr typename Q::Time us = 1.0e-6  * s;
    static constexpr typename Q::Time ns = 1.0e-9  * s;
    static constexpr typename Q::Time ps = 1.0e-12 * s;
    static constexpr typename Q::Time fs = 1.0e-15 * s;
    static constexpr typename Q::Time min = 60 * s;
    static constexpr typename Q::Time h = 3'600 * s;
    static constexpr typename Q::Time d = 24 * 60 * 60 * s;
    static constexpr typename Q::Time yr = 365.25 * 24 * 60 * 60 * s;
    static constexpr Quantity<Engine, 0, 0, 2, 0, 0, 0, 0> s2 = s * s; // to ease writing of other units, like F = A2 * s4 / kg / m2
    static constexpr Quantity<Engine, 0, 0, 3, 0, 0, 0, 0> s3 = s * s * s;
    static constexpr Quantity<Engine, 0, 0, 4, 0, 0, 0, 0> s4 = s * s * s * s;


    static constexpr typename Q::ElectricCurrent A = Q::ElectricCurrent::fromNumericalValue(1.0 * detail::ampere_v<Engine>);
    static constexpr typename Q::ElectricCurrent mA = 1.0e-3  * A;
    static constexpr typename Q::ElectricCurrent uA = 1.0e-6  * A;
    static constexpr typename Q::ElectricCurrent nA = 1.0e-9  * A;
    static constexpr typename Q::ElectricCurrent pA = 1.0e-12 * A;
    static constexpr typename Q::ElectricCurrent fA = 1.0e-15 * A;
    static constexpr typename Q::ElectricCurrent kA = 1.0e+3  * A;
    static constexpr typename Q::ElectricCurrent MA = 1.0e+6  * A;
    static constexpr typename Q::ElectricCurrent GA = 1.0e+9  * A;
    static constexpr typename Q::ElectricCurrent TA = 1.0e+12 * A;
    static constexpr Quantity<Engine, 0, 0, 0, 2, 0, 0, 0> A2 = A * A; // to ease writing of other units, like F = A2 * s4 / kg / m2
    static constexpr Quantity<Engine, 0, 0, 0, 3, 0, 0, 0> A3 = A * A * A;
    static constexpr Quantity<Engine, 0, 0, 0, 4, 0, 0, 0> A4 = A * A * A * A;

    static constexpr typename Q::Temperature K = Q::Temperature::fromNumericalValue(1.0 * detail::kelvin_v<Engine>);
    static constexpr typename Q::Temperature mK = 1.0e-3  * K;
    static constexpr typename Q::Temperature uK = 1.0e-6  * K;
    static constexpr typename Q::Temperature nK = 1.0e-9  * K;
    static constexpr typename Q::Temperature pK = 1.0e-12 * K;
    static constexpr typename Q::Temperature fK = 1.0e-15 * K;
    static constexpr typename Q::Temperature kK = 1.0e+3  * K;
    static constexpr typename Q::Temperature MK = 1.0e+6  * K;
    static constexpr typename Q::Temperature GK = 1.0e+9  * K;
    static constexpr typename Q::Temperature TK = 1.0e+12 * K;
    static constexpr Quantity<Engine, 0, 0, 0, 0, 2, 0, 0> K2 = K * K; // to ease writing of other units, like F = A2 * s4 / kg / m2
    static constexpr Quantity<Engine, 0, 0, 0, 0, 3, 0, 0> K3 = K * K * K;
    static constexpr Quantity<Engine, 0, 0, 0, 0, 4, 0, 0> K4 = K * K * K * K;
//     static constexpr typename Q::Temperature DegC = Q::Temperature::fromNumericalValue(273.15 + 1.0);
//     static constexpr typename Q::Temperature DegF = Q::Temperature::fromNumericalValue( (459.67 + 1.0) * (5/9) );

    static constexpr typename Q::AmountOfSubstance mol = Q::AmountOfSubstance::fromNumericalValue(1.0 * detail::mol_v<Engine>);
    static constexpr Quantity<Engine, 0, 0, 0, 0, 0, 2, 0> mol2 = mol * mol; // to ease writing of other units, like F = A2 * s4 / kg / m2
    static constexpr Quantity<Engine, 0, 0, 0, 0, 0, 3, 0> mol3 = mol * mol * mol;
    static constexpr Quantity<Engine, 0, 0, 0, 0, 0, 4, 0> mol4 = mol * mol * mol * mol;

    static constexpr typename Q::LuminousIntensity cd = Q::LuminousIntensity::fromNumericalValue(1.0 * detail::candela_v<Engine>);
    static constexpr Quantity<Engine, 0, 0, 0, 0, 0, 0, 2> cd2 = cd * cd; // to ease writing of other units, like F = A2 * s4 / kg / m2
    static constexpr Quantity<Engine, 0, 0, 0, 0, 0, 0, 3> cd3 = cd * cd * cd;
    static constexpr Quantity<Engine, 0, 0, 0, 0, 0, 0, 4> cd4 = cd * cd * cd * cd;

    static constexpr typename Q::Area m2 = m * m;
    static constexpr typename Q::Area cm2 = 1.0e-4 * m2;
    static constexpr typename Q::Area mm2 = 1.0e-6 * m2;
    static constexpr typename Q::Area a = 100 * m2;
    static constexpr typename Q::Area daa = 1000 * m2;
    static constexpr typename Q::Area ha = 10'000 * m2;
    static constexpr typename Q::Area km2 = 1'000'000 * m2;

    static constexpr typename Q::Volume m3 = m * m * m;
    static constexpr typename Q::Volume ml = 1e-6 * m3;
    static constexpr typename Q::Volume cl = 1e-5 * m3;
    static constexpr typename Q::Volume l = 1e-3 * m3;


    static constexpr typename Q::Velocity kmh = 1.0 / 3.6 * m / s;
    static constexpr typename Q::Acceleration Gal = 0.01 * m / s2; // galileo


    static constexpr double rad = 1.0; // radian
    static constexpr double sr = 1.0; // steradian
    static constexpr double deg = M_PI / 180.0 * rad; // degree


#define DEFINE_DERIVED_UNIT(Dimension, Unit, Definition) \
    static constexpr typename Q::Dimension Unit = Definition; \
    static constexpr typename Q::Dimension m##Unit = 1.0e-3  * Unit; \
    static constexpr typename Q::Dimension u##Unit = 1.0e-6  * Unit; \
    static constexpr typename Q::Dimension n##Unit = 1.0e-9  * Unit; \
    static constexpr typename Q::Dimension p##Unit = 1.0e-12 * Unit; \
    static constexpr typename Q::Dimension f##Unit = 1.0e-15 * Unit; \
    static constexpr typename Q::Dimension k##Unit = 1.0e+3  * Unit; \
    static constexpr typename Q::Dimension M##Unit = 1.0e+6  * Unit; \
    static constexpr typename Q::Dimension G##Unit = 1.0e+9  * Unit; \
    static constexpr typename Q::Dimension T##Unit = 1.0e+12 * Unit; \

    DEFINE_DERIVED_UNIT(Capacitance, F, A2 * s4 / kg / m2) // Farad
    DEFINE_DERIVED_UNIT(ElectricCharge, C, A * s) // Coulomb
    DEFINE_DERIVED_UNIT(CatalyticActivity, kat, mol / s) // Katal
    DEFINE_DERIVED_UNIT(DoseEquivalent, Sv, m2 / s2) // Sievert
    DEFINE_DERIVED_UNIT(ElectricalConductance, S, A2 * s3 / kg / m2) // Siemens
    DEFINE_DERIVED_UNIT(ElectricPotential, V, kg * m2 / A / s3) // Volt
#if defined(__clang__) || defined(_MSC_VER)
    DEFINE_DERIVED_UNIT(ElectricalResistance, Ω, kg * m2 / A2 / s3) // Ohm
#endif
    DEFINE_DERIVED_UNIT(ElectricalResistance, Ohm, kg * m2 / A2 / s3) // Ohm
    DEFINE_DERIVED_UNIT(Energy, J, kg * m2 / s2) // Joule
    DEFINE_DERIVED_UNIT(Energy, eV, 1.6021766208e-19 * J) // Electronvolt
    DEFINE_DERIVED_UNIT(Energy, Wh, 3'600 * J) // Watt hour
    DEFINE_DERIVED_UNIT(Force, N, kg * m / s2) // Newton
    DEFINE_DERIVED_UNIT(Frequency, Hz, 1.0 / s) // Hertz
    DEFINE_DERIVED_UNIT(Inductance, H, kg * m2 / A2 / s2) // Henry
    DEFINE_DERIVED_UNIT(Power, W, kg * m2 / s3) // Watt
    DEFINE_DERIVED_UNIT(MagneticFlux, Wb, kg * m2 / A / s2) // Weber
    DEFINE_DERIVED_UNIT(MagneticFluxDensity, T, kg / A / s2) // Tesla
    DEFINE_DERIVED_UNIT(Activity, Bq, 1.0 / s) // Becquerel
    DEFINE_DERIVED_UNIT(Illuminance, lx, cd * sr / m2) // Lux
    DEFINE_DERIVED_UNIT(LuminousFlux, lm, cd * sr) // Lumen
    DEFINE_DERIVED_UNIT(Dose, Gy, m2 / s2) // Gray
    DEFINE_DERIVED_UNIT(Pressure, Pa, kg / m / s2) // Pascal

    static constexpr typename Q::Pressure hPa = 1.0e+2  * Pa;
    static constexpr typename Q::Pressure bar = 1.0e+5   * Pa; // bar
    static constexpr typename Q::Pressure mbar = 1.0e+2  * Pa;
    static constexpr typename Q::Pressure ubar = 1.0e-1  * Pa;
    static constexpr typename Q::Pressure atm = 1'013.25 * hPa; // atmosphere
    static constexpr typename Q::Pressure Ba = 0.1 * Pa; // barye



    // Imperial units

    // Mass
    static constexpr typename Q::Mass gr = 64.798'91 * mg;
    static constexpr typename Q::Mass dr = 1.771'845'195'312'5 * g;
    static constexpr typename Q::Mass oz = 28.349'523'125 * g;
    static constexpr typename Q::Mass lb = 453.592'37 * g;
    static constexpr typename Q::Mass cwt = 45.359'237 * kg;
    static constexpr typename Q::Mass long_hundredweight = 50.802'345'44 * kg;
    static constexpr typename Q::Mass short_ton = 907.184'74 * kg;
    static constexpr typename Q::Mass long_ton = 1'016.046'908'8 * kg;

    // Length
    static constexpr typename Q::Length p = 0.3527777777777778 * mm;
    static constexpr typename Q::Length pica = 4.233333333333333 * mm;
    static constexpr typename Q::Length in = 25.4 * mm;
    static constexpr typename Q::Length ft = 0.3048 * m;
    static constexpr typename Q::Length yd = 0.9144 * m;
    static constexpr typename Q::Length mi = 1'609.344 * m;


    // Area
    static constexpr typename Q::Area ft2 = 0.092'903'41 * m2;
    static constexpr typename Q::Area ch2 = 404.6873 * m2;
    static constexpr typename Q::Area acre = 4'046.873 * m2;
    static constexpr typename Q::Area section = 2.589'998 * km2;
    static constexpr typename Q::Area twp = 93.239'93 * km2;

    // Volume - US customary units
    static constexpr typename Q::Volume minim = 61.611'519'921'875e-6 * l;
    static constexpr typename Q::Volume fldr = 3.696'691'195'312'5 * ml;
    static constexpr typename Q::Volume tsp = 4.928'921'593'75 * ml;
    static constexpr typename Q::Volume Tbsp = 14.786'764'781'25 * ml;
    static constexpr typename Q::Volume floz = 29.573'529'562'5 * ml;
    static constexpr typename Q::Volume jig = 44.360'294'343'75 * ml;
    static constexpr typename Q::Volume gi = 118.294'118'25 * ml;
    static constexpr typename Q::Volume cp = 236.588'236'5 * ml;
    static constexpr typename Q::Volume liquid_pint = 473.176'473 * ml;
    static constexpr typename Q::Volume liquid_quart = 0.946'352'946 * l;
    static constexpr typename Q::Volume liquid_gallon = 3.785'411'784 * l;
    static constexpr typename Q::Volume liquid_barrel = 119.240'471'196 * l;
    static constexpr typename Q::Volume oil_barrel = 158.987'294'928 * l;
    static constexpr typename Q::Volume hogshead = 238.480'942'392 * l;

    static constexpr typename Q::Volume dry_pint = 550.610'471'357'5 * ml;
    static constexpr typename Q::Volume dry_quart = 1.101'220'942'715 * l;
    static constexpr typename Q::Volume dry_gallon = 4.404'883'770'86 * l;
    static constexpr typename Q::Volume pk = 8.809'767'541'72 * l;
    static constexpr typename Q::Volume bu = 35.239'070'166'88 * l;
    static constexpr typename Q::Volume dry_barrel = 115.627'123'584 * l;





    // nautical
    static constexpr typename Q::Length ftm = 1.8288 * m;
    static constexpr typename Q::Length cb = 219.456 * m;
    static constexpr typename Q::Length nmi = 1'852.0 * m;
    static constexpr typename Q::Velocity kn = 1'852.0 * kmh;

};

template<typename Engine>
struct SI_constants_template {
    using units = SI_units_template<Engine>;
    using Q = typename Engine::SystemOfDimensions;

    // basic constants used for the numerics (ie. m0 == 1kg in SI, m0 == 1g in CGS and m0 == 1eV in natural units)
    static constexpr auto m0 = Q::Mass::fromNumericalValue(1.0);
    static constexpr auto s0 = Q::Length::fromNumericalValue(1.0);
    static constexpr auto t0 = Q::Time::fromNumericalValue(1.0);
    static constexpr auto I0 = Q::ElectricCurrent::fromNumericalValue(1.0);
    static constexpr auto T0 = Q::Temperature::fromNumericalValue(1.0);
    static constexpr auto n0 = Q::AmountOfSubstance::fromNumericalValue(1.0);
    static constexpr auto Iv0 = Q::LuminousFlux::fromNumericalValue(1.0);

    // Universal constants
    static constexpr auto Z_0  = 376.730313667 * units::Ohm; // characteristic impedance of vacuum
    static constexpr auto epsilon_0  = 8.854'187'817'620'39e-12 * units::F / units::m; // electric constant (vacuum permittivity)
    static constexpr auto mu_0  = 1.256'637'06e-6 * units::N / units::A2; // magnetic constant (vacuum permeability)
    static constexpr auto G  = 6.67408e-11 * units::m3 / units::kg / units::s2; // Newtonian constant of gravitation
    static constexpr auto h  = 6.62607e-34 * units::J * units::s; // Planck constant
    static constexpr auto h_bar  = 1.054'571'817e-34 * units::J * units::s; // reduced Planck constant
    static constexpr auto c_0 = 299'792'458 * units::m / units::s; // speed of light in vacuum

    // Electromagnetic constants
    static constexpr auto mu_B  = 9.2740100e-24 * units::J / units::T; // Bohr magneton
    static constexpr auto G_0  = 7.748091729e-5 * units::S; // conductance quantum
    static constexpr auto K_Jm90  = 48359e9 * units::Hz / units::V; // conventional value of Josephson constant[30]
    static constexpr auto R_Km90  = 25812.807 * units::Ohm; // conventional value of von Klitzing constant[32]
    static constexpr auto e  = 1.602176e-19 * units::C; // elementary charge
    static constexpr auto G_0_inv  = 12906.40372 * units::Ohm; // inverse conductance quantum
    static constexpr auto K_J  = 483597.8484e9 * units::Hz / units::V; // Josephson constant
    static constexpr auto Phi_0  = 2.067833848e-15 * units::Wb; // magnetic flux quantum
    static constexpr auto mu_N  = 5.0507837e-27 * units::J / units::T; // nuclear magneton
    static constexpr auto R_K  = 25812.80745 * units::Ohm; // von Klitzing constant
    static constexpr double k_e = 1.0 / (4*M_PI*epsilon_0);

    // Atomic and nuclear constants
    static constexpr auto a_0  = 5.29177210e-11 * units::m; // Bohr radius
    static constexpr auto r_e  = 2.8179403e-15 * units::m; // classical electron radius
    static constexpr auto g_e  = -2.00231930436256; // electron g-factor
    static constexpr auto m_e  = 9.1093837e-31 * units::kg; // electron mass
    static constexpr auto G0_F = 1.1663e-5 / units::GeV  / units::GeV; // Fermi coupling constant
    static constexpr auto alpha  = 7.2973525e-3; // fine-structure constant
    static constexpr auto E_h  = 4.3597447222e-18 * units::J; // Hartree energy
    static constexpr auto alpha_inv  = 137.035999084; // inverse fine-structure constant
    static constexpr auto m_p  = 1.67262192e-27 * units::kg; // proton mass
    static constexpr auto h_2me  = 3.6369475e-4 * units::m2 / units::s; // quantum of circulation
    static constexpr auto R_inf  = 10973731.568160 / units::m; // Rydberg constant
    static constexpr auto sigma_e  = 6.6524587e-29 * units::m2; // Thomson cross section
//     static constexpr auto θ_W  = ???; // weak mixing angle
    static constexpr auto sin2Theta_W  = 0.22290; // weak mixing angle

    // Physico-chemical constants
    static constexpr auto m_u  = 1.66053906e-27 * units::kg; // Atomic mass constant
    static constexpr auto N_A  = 6.02214e23 / units::mol; // Avogadro constant
    static constexpr auto k_B  = 1.380'649e-23 * units::J / units::K; // Boltzmann constant
    static constexpr auto F  = 96485.33212 * units::C / units::mol; // Faraday constant
    static constexpr auto c_1  = 3.741771852e-16 * units::W * units::m2; // first radiation constant
    static constexpr auto c_1L  = 1.191042972e-16 * units::W * units::m2 / units::sr; // first radiation constant for spectral radiance
    static constexpr auto n_0  = 2.651645804e25 / units::m3; // Loschmidt constant
    static constexpr auto R  = 8.314462618 * units::J / units::mol / units::K; // gas constant
    static constexpr auto N_Ah  = 3.990312712e-10 * units::J / units::Hz / units::mol; // molar Planck constant
    static constexpr auto M_u  = 0.99999999e-3 * units::kg / units::mol; // molar mass constant
    static constexpr auto V_m  = 22.71095464e-3 * units::m3 / units::mol; // molar volume of an ideal gas
    static constexpr auto S_0_by_R  = -1.15170753706; // Sackur–Tetrode constant
    static constexpr auto c_2  = 1.438776877e-2 * units::m * units::K; // second radiation constant
    static constexpr auto sigma  = 5.670374419e-8 * units::W / units::m2 / units::K2; // Stefan–Boltzmann constant
    static constexpr auto b  = 2.897771955e-3 * units::m * units::K; // Wien wavelength displacement law constant
    static constexpr auto b_prime = 5.878925757e10 * units::Hz / units::K; // Wien frequency displacement law constant

    // Adopted values
    static constexpr auto g_0  = 9.80665 * units::m / units::s2; // standard acceleration of gravity
    static constexpr auto atm  = 101325 * units::Pa; // standard atmosphere
    static constexpr auto Deltanu_Cs  = 9192631770 * units::Hz; // Caesium standard (defines the SI second)


#if defined(__clang__) || defined(_MSC_VER)
    static constexpr auto ε_0  = 8.8541878e-12 * units::F / units::m; // electric constant (vacuum permittivity)
    static constexpr auto μ_0  = 1.25663706e-6 * units::N / units::A2 ; // magnetic constant (vacuum permeability)
    static constexpr auto ℏ  = 1.054571817e-34 * units::J * units::s; // reduced Planck constant
    static constexpr auto μ_B  = 9.2740100e-24 * units::J / units::T; // Bohr magneton
    static constexpr auto Φ_0  = 2.067833848e-15 * units::Wb; // magnetic flux quantum
    static constexpr auto μ_N  = 5.0507837e-27 * units::J / units::T; // nuclear magneton
    static constexpr auto α  = 7.2973525e-3; // fine-structure constant
    static constexpr auto α_inv  = 137.035999084; // inverse fine-structure constant
    static constexpr auto σ_e  = 6.6524587e-29 * units::m2; // Thomson cross section
    static constexpr auto sin2θ_W  = 0.22290; // weak mixing angle
    static constexpr auto σ  = 5.670374419e-8 * units::W / units::m2 / units::K2; // Stefan–Boltzmann constant
    static constexpr auto Δν_Cs  = 9192631770 * units::Hz; // Caesium standard (defines the SI second)
#endif
};


template<typename Engn>
struct SiDimensions
{
    // Dimensions order: Mass, Length, Time, ElectricCurrent, Temperature, AmountOfSubstance, LuminousIntesity

    using Mass =              Quantity<Engn, 1, 0, 0, 0, 0, 0, 0>;
    using Length =            Quantity<Engn, 0, 1, 0, 0, 0, 0, 0>;
    using Time =              Quantity<Engn, 0, 0, 1, 0, 0, 0, 0>;
    using ElectricCurrent =   Quantity<Engn, 0, 0, 0, 1, 0, 0, 0>;
    using Temperature =       Quantity<Engn, 0, 0, 0, 0, 1, 0, 0>;
    using AmountOfSubstance = Quantity<Engn, 0, 0, 0, 0, 0, 1, 0>;
    using LuminousIntensity = Quantity<Engn, 0, 0, 0, 0, 0, 0, 1>;

    using Absement  = Quantity<Engn, 0, 1, 1, 0, 0, 0, 0>;
    using AbsorbedDoseRate  = Quantity<Engn, 0, 2, -3, 0, 0, 0, 0>;
    using Acceleration  = Quantity<Engn, 0, 1, -2, 0, 0, 0, 0>;
    using AngularAcceleration  = Quantity<Engn, 0, -2, -2, 0, 0, 0, 0>;
    using AngularMomentum  = Quantity<Engn, 1, 2, -1, 0, 0, 0, 0>;
    using AngularSpeed = Quantity<Engn, 0, 0, -1, 0, 0, 0, 0>;
    using Area  = Quantity<Engn, 0, 2, 0, 0, 0, 0, 0>;
    using AreaDensity  = Quantity<Engn, 1, -2, 0, 0, 0, 0, 0>;
    using Capacitance  = Quantity<Engn, -1, -2, 4, 2, 0, 0, 0>;
    using CatalyticActivity  = Quantity<Engn, 0, 0, -1, 0, 0, 1, 0>;
    using CatalyticActivityConcentration  = Quantity<Engn, 0, -3, -1, 0, 0, 1, 0>;
    using ChemicalPotential  = Quantity<Engn, 1, 2, -2, 0, 0, -1, 0>;
    using Crackle  = Quantity<Engn, 0, 1, -5, 0, 0, 0, 0>;
    using CurrentDensity  = Quantity<Engn, 0, -2, 0, 1, 0, 0, 0>;
    using DoseEquivalent  = Quantity<Engn, 0, 2, -2, 0, 0, 0, 0>;
    using DynamicViscosity  = Quantity<Engn, 1, -1, -1, 0, 0, 0, 0>;
    using ElectricCharge  = Quantity<Engn, 0, 0, 1, 1, 0, 0, 0>;
    using ElectricChargeDensity  = Quantity<Engn, 0, -3, 1, 1, 0, 0, 0>;
    using ElectricDisplacement  = Quantity<Engn, 0, -2, 1, 1, 0, 0, 0>;
    using ElectricFieldStrength  = Quantity<Engn, 1, 1, -3, -1, 0, 0, 0>;
    using ElectricalConductance  = Quantity<Engn, -1, -2, 3, 2, 0, 0, 0>;
    using ElectricalConductivity  = Quantity<Engn, -1, -3, 3, 2, 0, 0, 0>;
    using ElectricPotential  = Quantity<Engn, 1, 2, -3, -1, 0, 0, 0>;
    using ElectricalResistance  = Quantity<Engn, 1, 2, -3, -2, 0, 0, 0>;
    using ElectricalResistivity  = Quantity<Engn, 1, 3, -3, -2, 0, 0, 0>;
    using Energy  = Quantity<Engn, 1, 2, -2, 0, 0, 0, 0>;
    using EnergyDensity  = Quantity<Engn, 1, -1, -2, 0, 0, 0, 0>;
    using Entropy  = Quantity<Engn, 1, 2, -2, 0, -1, 0, 0>;
    using Force  = Quantity<Engn, 1, 1, -2, 0, 0, 0, 0>;
    using Frequency  = Quantity<Engn, 0, 0, -1, 0, 0, 0, 0>;
    using FuelEfficiency  = Quantity<Engn, 0, -2, 0, 0, 0, 0, 0>;
    using HalfLife  = Quantity<Engn, 0, 0, 1, 0, 0, 0, 0>;
    using Heat  = Quantity<Engn, 1, 2, -2, 0, 0, 0, 0>;
    using HeatCapacity  = Quantity<Engn, 1, 2, -2, 0, -1, 0, 0>;
    using HeatFluxDensity  = Quantity<Engn, 1, 0, -3, 0, 0, 0, 0>;
    using Illuminance  = Quantity<Engn, 0, -2, 0, 0, 0, 0, 1>;
    using Impedance  = Quantity<Engn, 1, 2, -3, -2, 0, 0, 0>;
    using Impulse  = Quantity<Engn, 1, 1, -1, 0, 0, 0, 0>;
    using Inductance  = Quantity<Engn, 1, 2, -2, -2, 0, 0, 0>;
    using Irradiance  = Quantity<Engn, 1, 0, -3, 0, 0, 0, 0>;
    using Intensity  = Quantity<Engn, 1, 0, -3, 0, 0, 0, 0>;
    using Jerk  = Quantity<Engn, 0, 1, -3, 0, 0, 0, 0>;
    using Jounce   = Quantity<Engn, 0, 1, -4, 0, 0, 0, 0>;
    using KinematicViscosity  = Quantity<Engn, 0, 2, -1, 0, 0, 0, 0>;
    using LinearDensity  = Quantity<Engn, 1, -1, 0, 0, 0, 0, 0>;
    using LuminousFlux   = Quantity<Engn, 0, 0, 0, 0, 0, 0, 1>;
    using MagneticFieldStrength  = Quantity<Engn, 0, -1, 0, 1, 0, 0, 0>;
    using MagneticFlux  = Quantity<Engn, 1, 2, -2, -1, 0, 0, 0>;
    using MagneticFluxDensity  = Quantity<Engn, 1, 0, -2, -1, 0, 0, 0>;
    using Magnetization  = Quantity<Engn, 0, -1, 0, 1, 0, 0, 0>;
    using MassDensity  = Quantity<Engn, 1, -3, 0, 0, 0, 0, 0>;
    using MeanLifetime  = Quantity<Engn, 0, 0, 1, 0, 0, 0, 0>;
    using MolarConcentration  = Quantity<Engn, 0, -3, 0, 0, 0, 1, 0>;
    using MolarEnergy  = Quantity<Engn, 1, 2, -2, 0, 0, -1, 0>;
    using MolarEntropy  = Quantity<Engn, 1, 2, -2, 0, -1, -1, 0>;
    using MolarHeatCapacity  = Quantity<Engn, 1, 2, -2, 0, -1, -1, 0>;
    using MomentOfInertia  = Quantity<Engn, 1, 2, 0, 0, 0, 0, 0>;
    using Momentum  = Quantity<Engn, 1, 1, -1, 0, 0, 0, 0>;
    using Permeability  = Quantity<Engn, 1, 1, -2, -2, 0, 0, 0>;
    using Permittivity  = Quantity<Engn, -1, -3, 4, 2, 0, 0, 0>;
    using Power  = Quantity<Engn, 1, 2, -3, 0, 0, 0, 0>;
    using Pressure  = Quantity<Engn, 1, -1, -2, 0, 0, 0, 0>;
    using Pop  = Quantity<Engn, 0, 1, -6, 0, 0, 0, 0>;
    using Activity  = Quantity<Engn, 0, 0, -1, 0, 0, 0, 0>;
    using Dose  = Quantity<Engn, 0, 2, -2, 0, 0, 0, 0>;
    using Radiance  = Quantity<Engn, 1, 0, -3, 0, 0, 0, 0>;
    using RadiantIntensity  = Quantity<Engn, 1, 2, -3, 0, 0, 0, 0>;
    using ReactionRate  = Quantity<Engn, 0, -3, -1, 0, 0, 1, 0>;
    using Reluctance  = Quantity<Engn, -1, -2, 2, 2, 0, 0, 0>;
    using Speed  = Quantity<Engn, 0, 1, -1, 0, 0, 0, 0>;
    using SpecificEnergy  = Quantity<Engn, 0, 2, -2, 0, 0, 0, 0>;
    using SpecificHeatCapacity  = Quantity<Engn, 0, 2, -2, 0, -1, 0, 0>;
    using SpecificVolume  = Quantity<Engn, -1, 3, 0, 0, 0, 0, 0>;
    using Spin  = Quantity<Engn, 1, 2, -1, 0, 0, 0, 0>;
    using Stress  = Quantity<Engn, 1, -1, -2, 0, 0, 0, 0>;
    using SurfaceTension  = Quantity<Engn, 1, 0, -2, 0, 0, 0, 0>;
    using TemperatureGradient  = Quantity<Engn, 0, -1, 0, 0, 1, 0, 0>;
    using ThermalConductivity  = Quantity<Engn, 1, 1, -3, 0, -1, 0, 0>;
    using Torque  = Quantity<Engn, 1, 2, -2, 0, 0, 0, 0>;
    using Velocity  = Quantity<Engn, 0, 1, -1, 0, 0, 0, 0>;
    using Volume  = Quantity<Engn, 0, 3, 0, 0, 0, 0, 0>;
    using VolumetricFlowRate  = Quantity<Engn, 0, 3, -1, 0, 0, 0, 0>;
    using Wavelength  = Quantity<Engn, 0, 1, 0, 0, 0, 0, 0>;
    using Wavenumber  = Quantity<Engn, 0, -1, 0, 0, 0, 0, 0>;
    using Wavevector  = Quantity<Engn, 0, -1, 0, 0, 0, 0, 0>;
    using Weight  = Quantity<Engn, 1, 1, -2, 0, 0, 0, 0>;
    using Work  = Quantity<Engn, 1, 2, -2, 0, 0, 0, 0>;
    using YoungsModulus  = Quantity<Engn, 1, -1, -2, 0, 0, 0, 0>;

};

struct SiEngine
{
    using NumberType = double;
    using SystemOfDimensions = SiDimensions<SiEngine>;
    using referenceEngine = void;

    static constexpr std::array<const char*, 7> unitStrings = { "m", "kg", "s", "A", "K", "mol", "cd" };
    static constexpr std::array<double, 7> baseUnits { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

    using units = SI_units_template<SiEngine>;
    using constants = SI_constants_template<SiEngine>;
};

using SI = SystemOfQuantities<SiEngine>;


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

    constexpr SI::LuminousIntensity operator"" _cd(long double value)        { return SI::LuminousIntensity::fromNumericalValue(value); }
    constexpr SI::LuminousIntensity operator"" _cd(unsigned long long value) { return SI::LuminousIntensity::fromNumericalValue(static_cast<long double>(value)); }

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

