/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright 2019  Anton Kreuzkamp <anton.kreuzkamp@kdab.com>
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

#include <quantityland2.h>
#include <si.h>
#include <cgs.h>
#include <natural.h>

#include <cmath>
#include <cassert>
#include <complex>

constexpr double pi = M_PI;

using namespace std::complex_literals;
using namespace Quantityland2;
using namespace Quantityland2::SI_literals;

// Multiplication
// static_assert(std::is_same_v<Quantity<void, 1, 0, 0> );

// Basic properties of Quantity
static_assert(std::movable<Quantity<void, 0, 1, 0>>);
static_assert(std::movable<Quantityland2::Quantity<Quantityland2::SiEngine, 0, 0, 0, 0, 1, 0, 0>>);

// Multiplication
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 0>>() * std::declval<Quantity<void, 0, 0, 0>>()),
    Quantity<void, 0, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 0, 0>>() * std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 0, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 0>>() * std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 0, 2, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 1, 0, 0>>() * std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 1, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 0>>() * std::declval<Quantity<void, 1, 0, 0>>()),
    Quantity<void, 1, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() * std::declval<Quantity<void, 1, 0, 0>>()),
    Quantity<void, 3, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() * std::declval<Quantity<void, 1, 3, 0>>()),
    Quantity<void, 3, 4, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 2>>() * std::declval<Quantity<void, 1, 3, 0>>()),
    Quantity<void, 1, 4, 2>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() * std::declval<Quantity<void, 0, 3, -1>>()),
    Quantity<void, 2, 4, -1>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() * std::declval<Quantity<void, -2, 3, 0>>()),
    Quantity<void, 0, 4, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 1, 1, 0>>() * std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 1, 2, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() * std::declval<Quantity<void, -2, -1, 0>>()),
    double
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, Ratio { 1, 2 }, Ratio { 3, 7 }, 0>>() * std::declval<Quantity<void, Ratio { 5, 10 }, Ratio { -8, 14 }, Ratio { -2, 5 } >>()),
    Quantity<void, 1, Ratio { -1, 7 }, Ratio { -2, 5 }>
>);

// Division
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 0>>() / std::declval<Quantity<void, 0, 0, 0>>()),
    Quantity<void, 0, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 0, 0>>() / std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 0, -1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 3, 0>>() / std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 0, 2, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 1, 0, 0>>() / std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 1, -1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 0>>() / std::declval<Quantity<void, 1, 0, 0>>()),
    Quantity<void, -1, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() / std::declval<Quantity<void, 1, 0, 0>>()),
    Quantity<void, 1, 1, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() / std::declval<Quantity<void, 1, 3, 0>>()),
    Quantity<void, 1, -2, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 0, 1, 2>>() / std::declval<Quantity<void, 1, 3, 0>>()),
    Quantity<void, -1, -2, 2>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() / std::declval<Quantity<void, 0, 3, -1>>()),
    Quantity<void, 2, -2, 1>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() / std::declval<Quantity<void, 2, 3, 0>>()),
    Quantity<void, 0, -2, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 1, -1, 0>>() / std::declval<Quantity<void, 0, 1, 0>>()),
    Quantity<void, 1, -2, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, 1, 0>>() / std::declval<Quantity<void, 2, 1, 0>>()),
    double
>);
static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, Ratio { 1, 2 }, Ratio { 3, 7 }, 0>>() / std::declval<Quantity<void, Ratio { 1, 2 }, Ratio { 8, 14 }, Ratio { -2, 5 } >>()),
    Quantity<void, 0, Ratio { -1, 7 }, Ratio { 2, 5 }>
>);


// Multiplication / Division with/by scalars

static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, Ratio{1, 2}, 0>>() * std::declval<double>()),
    Quantity<void, 2, Ratio{1, 2}, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<double>() * std::declval<Quantity<void, 2, Ratio{1, 2}, 0>>()),
    Quantity<void, 2, Ratio{1, 2}, 0>
>);

static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, Ratio{1, 2}, 0> >() / std::declval<double>()),
    Quantity<void, 2, Ratio{1, 2}, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<double>() / std::declval<Quantity<void, 2, Ratio{1, 2}, 0> >()),
    Quantity<void, -2, Ratio {-1, 2}, 0>
>);

static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, Ratio{1, 2}, 0>>() * std::declval<int>()),
    Quantity<void, 2, Ratio{1, 2}, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<int>() * std::declval<Quantity<void, 2, Ratio{1, 2}, 0> >()),
    Quantity<void, 2, Ratio{1, 2}, 0>
>);

static_assert(std::is_same_v<
    decltype(std::declval<Quantity<void, 2, Ratio{1, 2}, 0> >() / std::declval<int>()),
    Quantity<void, 2, Ratio{1, 2}, 0>
>);
static_assert(std::is_same_v<
    decltype(std::declval<int>() / std::declval<Quantity<void, 2, Ratio{1, 2}, 0> >()),
    Quantity<void, -2, Ratio {-1, 2}, 0>
>);

// Addition and substraction
template<typename First_t, typename Second_t> using second_t = Second_t;

template<typename ...Args>
auto canAdd(Args ...args) -> std::false_type;
template<typename T, typename U>
auto canAdd(T t, U u) -> second_t<decltype(t + u), std::true_type>;
template<typename T, typename U>
constexpr bool canAdd_v = decltype(canAdd(std::declval<T>(), std::declval<U>()))::value;


template<typename ...Args>
auto canSubtract(Args ...args) -> std::false_type;
template<typename T, typename U>
auto canSubtract(T t, U u) -> second_t<decltype(t - u), std::true_type>;
template<typename T, typename U>
constexpr bool canSubtract_v = decltype(canSubtract(std::declval<T>(), std::declval<U>()))::value;

template<typename ...Args>
auto canAssign(Args ...args) -> std::false_type;
template<typename T, typename U>
auto canAssign(T t, U u) -> second_t<decltype(t = u), std::true_type>;
template<typename T, typename U>
constexpr bool canAssign_v = decltype(canAssign(std::declval<T>(), std::declval<U>()))::value;

static_assert(canAdd_v<SI::Mass, SI::Mass>);
static_assert(canAdd_v<SI::Mass, CGS::Mass>);
// static_assert(!canAdd_v<SI::Mass, SI::Length>);

// Test CGS
constexpr CGS::Mass m1 = 1 * CGS::units::kg;
static_assert(m1.numericalValue() == 1000.0);
constexpr CGS::Length s1 = 3.14 * CGS::units::cm;
static_assert(s1.numericalValue() == 3.14);
constexpr CGS::Volume v1 = 1 * CGS::units::m3;
static_assert(v1.numericalValue() == 1'000'000);
static_assert(CGS::units::m.numericalValue() == 100);
static_assert(CGS::units::s.numericalValue() == 1);
static_assert((CGS::units::m / CGS::units::s).numericalValue() == 100);
static_assert(CGS::constants::c_0.numericalValue() == 29'979'245'800);



// Conversion
constexpr CGS::Mass m = 1_kg;
static_assert(m.numericalValue() == 1000.0);

constexpr CGS::Length cgsL = CGS::Length::fromNumericalValue(5.0); // 5cm
constexpr SI::Length siL = cgsL;
static_assert(siL.numericalValue() == 0.05);

// Calculations

static_assert( 5_kg * 3.0 == 15_kg);
static_assert( 5_kg * 3.0_m == 15*(SI::units::kg * SI::units::m));
static_assert( 6_kg / 2_kg == 3 );

constexpr auto kepler(SI::Mass m, SI::Length a) {
    return 4*pi*pi * a*a*a / ( SI::constants::G * m );
}
void piTimesTenToTheSeven() {
    auto sunMass = 1.9884e33 * CGS::units::g;
    auto earthMass = 5.9723e24_kg;
    auto a = 149597870700_m;
    std::cout << "A year lasts " << root<2>(kepler(sunMass + earthMass, a)) << std::endl;

    std::cout << "sunMass: " << sunMass << std::endl;
    std::cout << "earthMass: " << earthMass << std::endl;
    std::cout << "a: " << a << std::endl;
    std::cout << "4*pi*pi: " << (4*pi*pi) << std::endl;
    std::cout << "a*a*a: " << (4*pi*pi*a*a*a) << std::endl;
    std::cout << "4*pi*pi*a*a*a: " << (a*a*a) << std::endl;
    std::cout << "G: " << (SI::constants::G) << std::endl;
    std::cout << "G * sunMass: " << (SI::constants::G * sunMass) << std::endl;
    auto Tsq = 4*pi*pi * a*a*a / ( SI::constants::G * sunMass );
    std::cout << "Tsq: " << Tsq << std::endl;
    std::cout << "A year lasts " << root<2>(Tsq) << std::endl;
}


// Test natural units
constexpr double fuzzy_equal(double lhs, double rhs, double fuzzy_threshold = 1.0e-6)
{
    double diff = rhs - lhs;
    double sum = rhs + lhs;
    if (diff > 0) return (diff / sum) < fuzzy_threshold;
    else return (-diff / sum) < fuzzy_threshold;
}

constexpr NaturalUnits::Mass nu_m = 5.0 * NaturalUnits::units::eV;
constexpr NaturalUnits::Length nu_l = 5.0 / NaturalUnits::units::eV;
constexpr NaturalUnits::Time nu_t = nu_l;
static_assert(canAssign_v<NaturalUnits::Length, NaturalUnits::Time>);
static_assert(!canAssign_v<NaturalUnits::Mass, NaturalUnits::Time>);
static_assert(nu_l.numericalValue() == nu_t.numericalValue());

constexpr Quantity<SiEngine, -1, 0, 0, 0, 0, 0, 0> tmp = NaturalUnits::toSiDimensions<Quantity<SiEngine, -1, 0, 0, 0, 0, 0, 0>>(nu_l);
static_assert(tmp.numericalValue() != nu_l.numericalValue());
constexpr auto si_l_man = tmp * SI::constants::h_bar / SI::constants::c_0;
constexpr auto si_l_auto = NaturalUnits::toSiDimensions<SI::Length>(nu_l);
constexpr auto cgs_l_auto = NaturalUnits::toSiDimensions<CGS::Length>(nu_l);
constexpr NaturalUnits::Length nu_l2 = NaturalUnits::fromSiDimensions(si_l_man / SI::constants::h_bar * SI::constants::c_0);
constexpr NaturalUnits::Length nu_l3 = NaturalUnits::fromSiDimensions(si_l_man);
constexpr NaturalUnits::Length nu_l4 = NaturalUnits::fromSiDimensions(si_l_auto / SI::constants::h_bar * SI::constants::c_0);
constexpr NaturalUnits::Length nu_l5 = NaturalUnits::fromSiDimensions(si_l_auto);
constexpr NaturalUnits::Time minInNU_auto = NaturalUnits::fromSiDimensions(SI::units::min);
constexpr NaturalUnits::Time minInNU_man = NaturalUnits::fromSiDimensions(SI::units::min / SI::constants::h_bar * SI::constants::c_0 * SI::constants::c_0);

constexpr auto nonIntegerMassDimension = Quantity<NaturalEngine, Ratio { -13, 15 }>::fromNumericalValue(1.5);
constexpr Quantity<SiEngine, Ratio {-1, 5}, Ratio {2, 3}, 0, 0, 0, 0, 0> nonIntegerSiQuantity = NaturalUnits::toSiDimensions<Quantity<SiEngine, Ratio {-1, 5}, Ratio {2, 3}, 0, 0, 0, 0, 0>>(nonIntegerMassDimension);

static_assert(fuzzy_equal(NaturalUnits::fromSiDimensions(SI::units::eV).numericalValue(), 1.0));
static_assert(fuzzy_equal(nu_l.numericalValue(), nu_l2.numericalValue())); // converting to si and back manually gives the original value
static_assert(fuzzy_equal(nu_l.numericalValue(), nu_l3.numericalValue())); // converting to si manually and automatically back gives the original value
static_assert(fuzzy_equal(nu_l.numericalValue(), nu_l4.numericalValue())); // converting to si automatically and back manually gives the original value
static_assert(fuzzy_equal(nu_l.numericalValue(), nu_l5.numericalValue())); // converting to si and back automatically gives the original value
static_assert(fuzzy_equal(si_l_auto.numericalValue(), si_l_man.numericalValue())); // automatic and manual conversion to si are identical
static_assert(fuzzy_equal(100 * si_l_auto.numericalValue(), cgs_l_auto.numericalValue())); // automatic conversion to SI-derived Engine works as expected
static_assert(fuzzy_equal(minInNU_auto.numericalValue(), minInNU_man.numericalValue())); // automatic and manual conversion from si are identical
static_assert(fuzzy_equal(minInNU_auto.numericalValue(), minInNU_man.numericalValue())); //

static_assert(NaturalUnits::toSiDimensions<SI::Mass>(1.0 * NaturalUnits::units::eV).numericalValue() == 1.782662e-36);
static_assert(NaturalUnits::toSiDimensions<SI::Length>(1.0 / NaturalUnits::units::eV).numericalValue() == 1.97327e-7);
static_assert(NaturalUnits::toSiDimensions<SI::Time>(1.0 / NaturalUnits::units::eV).numericalValue() == 6.582119e-16);
static_assert(NaturalUnits::toSiDimensions<SI::Temperature>(1.0 * NaturalUnits::units::eV).numericalValue() == 1.1604505e4);
static_assert(fuzzy_equal(NaturalUnits::toSiDimensions<SI::Force>(1.0 * NaturalUnits::units::eV * NaturalUnits::units::eV).numericalValue(), 1.782662e-36 * 1.97327e-7 / 6.582119e-16 / 6.582119e-16));

constexpr auto cmInNU = NaturalUnits::fromSiDimensions(SI::units::cm);
constexpr auto cmInNU2 = NaturalUnits::fromSiDimensions(CGS::units::cm);
constexpr auto cminpereV = 1.0 * SI::units::cm / SI::constants::h_bar / SI::constants::c_0 * SI::units::eV;
static_assert(fuzzy_equal(cmInNU.numericalValue(), cminpereV));
static_assert(fuzzy_equal(cmInNU.numericalValue(), cmInNU2.numericalValue()));

void relativisticKinematic()
{
    std::cout << "====Relativistic Kinematic====\n";
    double beta1 = 0.990;
    double beta2 = 0.999;
    double gamma1 = 1.0 / std::sqrt(1 - beta1*beta1);
    double gamma2 = 1.0 / std::sqrt(1 - beta2*beta2);
    NaturalUnits::Mass me = 0.511e6  * NaturalUnits::units::eV;
    NaturalUnits::Momentum p1 = gamma1 * me * beta1;
    NaturalUnits::Momentum p2 = gamma2 * me * beta2;
    NaturalUnits::Energy E1 = gamma1 * me;
    NaturalUnits::Energy E2 = gamma2 * me;
    NaturalUnits::Energy deltaE1 = p2 - p1;
    NaturalUnits::Energy deltaE2 = E2 - E1;
    std::cout << "deltaE1 == " << (deltaE1 / NaturalUnits::units::eV) << "eV." << std::endl;
    std::cout << "deltaE1 == " << (NaturalUnits::toSiDimensions<SI::Energy>(deltaE1) / SI::units::eV) << "eV." << std::endl;
    std::cout << "deltaE2 == " << (deltaE2 / NaturalUnits::units::eV) << "eV." << std::endl;
    std::cout << "deltaE2 == " << (NaturalUnits::toSiDimensions<SI::Energy>(deltaE2) / SI::units::eV) << "eV." << std::endl;
//     assert(std::abs(deltaE / NaturalUnits::units::eV - NaturalUnits::toSiDimensions<SI::Energy>(deltaE) / SI::units::eV) / (deltaE / NaturalUnits::units::eV) < 0.000'001);
}

// constexpr NaturalUnits::Energy E =

int main() {
    std::cout << "Natural Units conversion: " << minInNU_auto << " " << minInNU_man << " " << std::endl;

    // Test cout
    std::cout << 100_m << std::endl;
    std::cout << 100 * CGS::units::cm << std::endl;

    std::cout << (6_kg / 3_kg) << std::endl;
    std::cout << (100_m /  9.58_s) << std::endl;

    piTimesTenToTheSeven();

    std::cout << "====Failures===" << std::endl;
    std::cout << "tmp: " << tmp << std::endl;
    std::cout << "si_l: " << si_l_man << std::endl;

    std::cout << (1.0 * NaturalUnits::units::eV * NaturalUnits::units::eV).numericalValue() << std::endl;

    relativisticKinematic();

    assert(root<2>(5_J)*root<2>(5_J) == 5_J);
    Quantity<CGSEngine, 0, Ratio {1, 2}, 0, 0, 0, 0, 0> nonIntegerCGS = root<2>(4*SI::units::m);
    assert(nonIntegerCGS / root<2>(SI::units::m) == 2.0);
    assert(nonIntegerCGS.numericalValue() == 2.0);

    return 0;
}



// Test own Engine
struct TestEngine : public SiDimensions<TestEngine>
{
    using referenceEngine = SiEngine;
    static constexpr std::tuple baseUnits {1.9884e30_kg, 149'597'870.7_km, 1.0, 1.0, 1.0, 1.0, 1.0 };
};

constexpr CGS::Length cgsA = CGS::Length::fromNumericalValue(29'919'574'140'000);
constexpr TestEngine::Length testEngnA = cgsA;
static_assert(testEngnA.numericalValue() == 2.0);

constexpr TestEngine::Mass testEngnM = TestEngine::Mass::fromNumericalValue(10.0);
constexpr CGS::Mass cgsM = testEngnM;
static_assert(cgsM.numericalValue() == 1.9884e34);

// Test float number support

struct FloatSiEngine
{
    using NumberType = float;
    using SystemOfDimensions = SiDimensions<FloatSiEngine>;

    static constexpr std::tuple baseUnits { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

    using units = SI_units_template<FloatSiEngine>;
    using constants = SI_constants_template<FloatSiEngine>;
};
using FloatSI = SystemOfQuantities<FloatSiEngine>;

FloatSI::Length floatLength = 1.4f * FloatSI::units::m;
static_assert(std::is_same_v<numberTypeOf_t<SI>, double>);
static_assert(std::is_same_v<numberTypeOf_t<FloatSiEngine>, float>);

// Test complex number support


struct ComplexSiEngine
{
    using NumberType = std::complex<double>;
    using SystemOfDimensions = SiDimensions<ComplexSiEngine>;

    static constexpr std::tuple baseUnits { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

    using units = SI_units_template<ComplexSiEngine>;
    using constants = SI_constants_template<ComplexSiEngine>;
};
using ComplexSI = SystemOfQuantities<ComplexSiEngine>;

static_assert(std::is_same_v<numberTypeOf_t<ComplexSI>, std::complex<double>>);

ComplexSI::Length complexDistance = (1.4 + 9.6i) * ComplexSI::units::m;
ComplexSI::Time complexDuration = (2.8 + 9.6i) * ComplexSI::units::s;
auto complexVelocity = complexDistance / complexDuration;


class MySimulation : private SI::units
{
    void foo() {
        auto x = 1 * m / s2;
        std::cout << x;
    }
};


// Test non-integer exponents
static_assert(std::is_same_v<decltype(root<2>(1_kg)), Quantity<SiEngine, Ratio{1, 2}, 0, 0, 0, 0, 0, 0>>);
static_assert(std::is_same_v<decltype(root<2>(1_kg) * root<2>(1_kg)), Quantity<SiEngine, 1, 0, 0, 0, 0, 0, 0>>);
Quantity<SiEngine, Ratio{1, 2}, 0, 0, 0, 0, 0, 0> sqrtMass = root<2>(1_kg);
SI::Mass sqrtMassSquared = sqrtMass * sqrtMass;
