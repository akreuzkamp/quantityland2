#include <quantityland2.h>
#include <si.h>
#include <cgs.h>
#include <natural.h>

#include <cmath>
#include <cassert>

constexpr double pi = M_PI;


using namespace Quantityland2;
using namespace Quantityland2::base_dimensions;
using namespace Quantityland2::SI_literals;


// Multiplication
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>>,
    Quantity<void> >,
    Quantity<void, DimensionComponent<Length, 1>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void>,
    Quantity<void, DimensionComponent<Length, 1>> >,
    Quantity<void, DimensionComponent<Length, 1>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>>,
    Quantity<void, DimensionComponent<Length, 1>> >,
    Quantity<void, DimensionComponent<Length, 2>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Mass, 1>>,
    Quantity<void, DimensionComponent<Length, 1>> >,
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 1>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>>,
    Quantity<void, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 1>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
    Quantity<void, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 3>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
    Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, 4>, DimensionComponent<Mass, 3>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Time, 2>>,
    Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, 4>, DimensionComponent<Mass, 1>, DimensionComponent<Time, 2>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
    Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Time, -1>> >,
    Quantity<void, DimensionComponent<Length, 4>, DimensionComponent<Mass, 2>, DimensionComponent<Time, -1>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
        Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Mass, -2>> >,
        Quantity<void, DimensionComponent<Length, 4>>
>);
static_assert(std::is_same_v<detail::mergeMul_t<
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
        Quantity<void, DimensionComponent<Length, -1>, DimensionComponent<Mass, -2>> >,
        double
>);

// Division
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 1>>,
    Quantity<void> >,
    Quantity<void, DimensionComponent<Length, 1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void>,
    Quantity<void, DimensionComponent<Length, 1>> >,
    Quantity<void, DimensionComponent<Length, -1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 3>>,
    Quantity<void, DimensionComponent<Length, 1>> >,
    Quantity<void, DimensionComponent<Length, 2>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Mass, 1>>,
    Quantity<void, DimensionComponent<Length, 1>> >,
    Quantity<void, DimensionComponent<Length, -1>, DimensionComponent<Mass, 1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 1>>,
    Quantity<void, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, -1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
    Quantity<void, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
    Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, -2>, DimensionComponent<Mass, 1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Time, 2>>,
    Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Mass, 1>> >,
    Quantity<void, DimensionComponent<Length, -2>, DimensionComponent<Mass, -1>, DimensionComponent<Time, 2>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
    Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
    Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Time, -1>> >,
    Quantity<void, DimensionComponent<Length, -2>, DimensionComponent<Mass, 2>, DimensionComponent<Time, 1>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
        Quantity<void, DimensionComponent<Length, 3>, DimensionComponent<Mass, 2>> >,
        Quantity<void, DimensionComponent<Length, -2>>
>);
static_assert(std::is_same_v<detail::mergeDiv_t<
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>,
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >,
        double
>);


// Multiplication / Division with/by scalars

static_assert(std::is_same_v<decltype(
            std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >() * std::declval<double>()
        ),
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>
>);
static_assert(std::is_same_v<decltype(
            std::declval<double>() * std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >()
        ),
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>
>);

static_assert(std::is_same_v<decltype(
            std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >() / std::declval<double>()
        ),
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>
>);
static_assert(std::is_same_v<decltype(
            std::declval<double>() / std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >()
        ),
        Quantity<void, DimensionComponent<Length, -1>, DimensionComponent<Mass, -2>>
>);

static_assert(std::is_same_v<decltype(
            std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >() * std::declval<int>()
        ),
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>
>);
static_assert(std::is_same_v<decltype(
            std::declval<int>() * std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >()
        ),
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>
>);

static_assert(std::is_same_v<decltype(
            std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >() / std::declval<int>()
        ),
        Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>>
>);
static_assert(std::is_same_v<decltype(
            std::declval<int>() / std::declval<Quantity<void, DimensionComponent<Length, 1>, DimensionComponent<Mass, 2>> >()
        ),
        Quantity<void, DimensionComponent<Length, -1>, DimensionComponent<Mass, -2>>
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
static_assert(CGS::constants::c0.numericalValue() == 29'979'245'800);


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
constexpr NaturalUnits::Mass nu_m = 5.0 * NaturalUnits::units::eV;
constexpr NaturalUnits::Length nu_l = 5.0 / NaturalUnits::units::eV;
constexpr NaturalUnits::Time nu_t = nu_l;
static_assert(canAssign_v<NaturalUnits::Length, NaturalUnits::Time>);
static_assert(!canAssign_v<NaturalUnits::Mass, NaturalUnits::Time>);
static_assert(nu_l.numericalValue() == nu_t.numericalValue());

constexpr Quantity<SI, DimensionComponent<Mass, -1>> tmp = nu_l;
static_assert(tmp.numericalValue() != nu_l.numericalValue());
constexpr SI::Length si_l = tmp * SI::constants::hbar / SI::constants::c0;
// static_assert(si_l == 1.97e-7_m);
static_assert(convertToSI<SI::Mass>(1.0 * NaturalUnits::units::eV).numericalValue() == 1.782662e-36);
static_assert(convertToSI<SI::Length>(1.0 / NaturalUnits::units::eV).numericalValue() == 1.97327e-7);
static_assert(convertToSI<SI::Time>(1.0 / NaturalUnits::units::eV).numericalValue() == 6.582119e-16);
static_assert(convertToSI<SI::Temperature>(1.0 * NaturalUnits::units::eV).numericalValue() == 1.1604505e4);
// static_assert(convertToSI<SI::Force>(1.0 * NaturalUnits::units::eV * NaturalUnits::units::eV).numericalValue() == 1.78e-36 * 1.97e-7 / 6.58e-16 / 6.58e-16);
// constexpr auto asoiuoiudf = convertToSI<SI::Length>(1.0 * NaturalUnits::units::eV);
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
    std::cout << "deltaE1 == " << (convertToSI<SI::Energy>(deltaE1) / SI::units::eV) << "eV." << std::endl;
    std::cout << "deltaE2 == " << (deltaE2 / NaturalUnits::units::eV) << "eV." << std::endl;
    std::cout << "deltaE2 == " << (convertToSI<SI::Energy>(deltaE2) / SI::units::eV) << "eV." << std::endl;
//     assert(std::abs(deltaE / NaturalUnits::units::eV - convertToSI<SI::Energy>(deltaE) / SI::units::eV) / (deltaE / NaturalUnits::units::eV) < 0.000'001);
}

// constexpr NaturalUnits::Energy E =

int main() {
    // Test cout
    std::cout << 100_m << std::endl;
    std::cout << 100 * CGS::units::cm << std::endl;

    std::cout << (6_kg / 3_kg) << std::endl;
    std::cout << (100_m /  9.58_s) << std::endl;

    piTimesTenToTheSeven();

    std::cout << "====Failures===" << std::endl;
    std::cout << "tmp: " << tmp << std::endl;
    std::cout << "si_l: " << si_l << std::endl;

    std::cout << (1.0 * NaturalUnits::units::eV * NaturalUnits::units::eV).numericalValue() << std::endl;

    relativisticKinematic();

    return 0;
}



// Test own Engine
namespace TestEngineConfigDetail {
    template<typename T> constexpr auto baseUnit = 1.0;
    template<> constexpr SI::Length baseUnit<typename base_dimensions::Length> = 149'597'870.7_km;
    template<> constexpr SI::Mass baseUnit<typename base_dimensions::Mass> = 1.9884e30_kg;
}

struct TestEngineConfig
{
    using referenceEngine = SI;
    template<typename T> static constexpr auto baseUnit = TestEngineConfigDetail::baseUnit<T>;
};
using TestEngine = Engine<TestEngineConfig, SiDimensions>;

constexpr CGS::Length cgsA = CGS::Length::fromNumericalValue(29'919'574'140'000);
constexpr TestEngine::Length testSouA = cgsA;
static_assert(testSouA.numericalValue() == 2.0);

constexpr TestEngine::Mass testSouM = TestEngine::Mass::fromNumericalValue(10.0);
constexpr CGS::Mass cgsM = testSouM;
static_assert(cgsM.numericalValue() == 1.9884e34);

// Test

static_assert(verifyDimensionOrder_v<SI::Absement                      >);
static_assert(verifyDimensionOrder_v<SI::AbsorbedDoseRate              >);
static_assert(verifyDimensionOrder_v<SI::Acceleration                  >);
static_assert(verifyDimensionOrder_v<SI::AngularAcceleration           >);
static_assert(verifyDimensionOrder_v<SI::AngularMomentum               >);
static_assert(verifyDimensionOrder_v<SI::AngularSpeed                  >);
static_assert(verifyDimensionOrder_v<SI::Area                          >);
static_assert(verifyDimensionOrder_v<SI::AreaDensity                   >);
static_assert(verifyDimensionOrder_v<SI::Capacitance                   >);
static_assert(verifyDimensionOrder_v<SI::CatalyticActivity             >);
static_assert(verifyDimensionOrder_v<SI::CatalyticActivityConcentration>);
static_assert(verifyDimensionOrder_v<SI::ChemicalPotential             >);
static_assert(verifyDimensionOrder_v<SI::Crackle                       >);
static_assert(verifyDimensionOrder_v<SI::CurrentDensity                >);
static_assert(verifyDimensionOrder_v<SI::DoseEquivalent                >);
static_assert(verifyDimensionOrder_v<SI::DynamicViscosity              >);
static_assert(verifyDimensionOrder_v<SI::ElectricCharge                >);
static_assert(verifyDimensionOrder_v<SI::ElectricChargeDensity         >);
static_assert(verifyDimensionOrder_v<SI::ElectricDisplacement          >);
static_assert(verifyDimensionOrder_v<SI::ElectricFieldStrength         >);
static_assert(verifyDimensionOrder_v<SI::ElectricalConductance         >);
static_assert(verifyDimensionOrder_v<SI::ElectricalConductivity        >);
static_assert(verifyDimensionOrder_v<SI::ElectricPotential             >);
static_assert(verifyDimensionOrder_v<SI::ElectricalResistance          >);
static_assert(verifyDimensionOrder_v<SI::ElectricalResistivity         >);
static_assert(verifyDimensionOrder_v<SI::Energy                        >);
static_assert(verifyDimensionOrder_v<SI::EnergyDensity                 >);
static_assert(verifyDimensionOrder_v<SI::Entropy                       >);
static_assert(verifyDimensionOrder_v<SI::Force                         >);
static_assert(verifyDimensionOrder_v<SI::Frequency                     >);
static_assert(verifyDimensionOrder_v<SI::FuelEfficiency                >);
static_assert(verifyDimensionOrder_v<SI::HalfLife                      >);
static_assert(verifyDimensionOrder_v<SI::Heat                          >);
static_assert(verifyDimensionOrder_v<SI::HeatCapacity                  >);
static_assert(verifyDimensionOrder_v<SI::HeatFluxDensity               >);
static_assert(verifyDimensionOrder_v<SI::Illuminance                   >);
static_assert(verifyDimensionOrder_v<SI::Impedance                     >);
static_assert(verifyDimensionOrder_v<SI::Impulse                       >);
static_assert(verifyDimensionOrder_v<SI::Inductance                    >);
static_assert(verifyDimensionOrder_v<SI::Irradiance                    >);
static_assert(verifyDimensionOrder_v<SI::Intensity                     >);
static_assert(verifyDimensionOrder_v<SI::Jerk                          >);
static_assert(verifyDimensionOrder_v<SI::Jounce                        >);
static_assert(verifyDimensionOrder_v<SI::KinematicViscosity            >);
static_assert(verifyDimensionOrder_v<SI::LinearDensity                 >);
static_assert(verifyDimensionOrder_v<SI::LuminousFlux                  >);
static_assert(verifyDimensionOrder_v<SI::MagneticFieldStrength         >);
static_assert(verifyDimensionOrder_v<SI::MagneticFlux                  >);
static_assert(verifyDimensionOrder_v<SI::MagneticFluxDensity           >);
static_assert(verifyDimensionOrder_v<SI::Magnetization                 >);
static_assert(verifyDimensionOrder_v<SI::MassDensity                   >);
static_assert(verifyDimensionOrder_v<SI::MeanLifetime                  >);
static_assert(verifyDimensionOrder_v<SI::MolarConcentration            >);
static_assert(verifyDimensionOrder_v<SI::MolarEnergy                   >);
static_assert(verifyDimensionOrder_v<SI::MolarEntropy                  >);
static_assert(verifyDimensionOrder_v<SI::MolarHeatCapacity             >);
static_assert(verifyDimensionOrder_v<SI::MomentOfInertia               >);
static_assert(verifyDimensionOrder_v<SI::Momentum                      >);
static_assert(verifyDimensionOrder_v<SI::Permeability                  >);
static_assert(verifyDimensionOrder_v<SI::Permittivity                  >);
static_assert(verifyDimensionOrder_v<SI::Power                         >);
static_assert(verifyDimensionOrder_v<SI::Pressure                      >);
static_assert(verifyDimensionOrder_v<SI::Pop                           >);
static_assert(verifyDimensionOrder_v<SI::Activity                      >);
static_assert(verifyDimensionOrder_v<SI::Dose                          >);
static_assert(verifyDimensionOrder_v<SI::Radiance                      >);
static_assert(verifyDimensionOrder_v<SI::RadiantIntensity              >);
static_assert(verifyDimensionOrder_v<SI::ReactionRate                  >);
static_assert(verifyDimensionOrder_v<SI::Reluctance                    >);
static_assert(verifyDimensionOrder_v<SI::Speed                         >);
static_assert(verifyDimensionOrder_v<SI::SpecificEnergy                >);
static_assert(verifyDimensionOrder_v<SI::SpecificHeatCapacity          >);
static_assert(verifyDimensionOrder_v<SI::SpecificVolume                >);
static_assert(verifyDimensionOrder_v<SI::Spin                          >);
static_assert(verifyDimensionOrder_v<SI::Stress                        >);
static_assert(verifyDimensionOrder_v<SI::SurfaceTension                >);
static_assert(verifyDimensionOrder_v<SI::TemperatureGradient           >);
static_assert(verifyDimensionOrder_v<SI::ThermalConductivity           >);
static_assert(verifyDimensionOrder_v<SI::Torque                        >);
static_assert(verifyDimensionOrder_v<SI::Velocity                      >);
static_assert(verifyDimensionOrder_v<SI::Volume                        >);
static_assert(verifyDimensionOrder_v<SI::VolumetricFlowRate            >);
static_assert(verifyDimensionOrder_v<SI::Wavelength                    >);
static_assert(verifyDimensionOrder_v<SI::Wavenumber                    >);
static_assert(verifyDimensionOrder_v<SI::Wavevector                    >);
static_assert(verifyDimensionOrder_v<SI::Weight                        >);
static_assert(verifyDimensionOrder_v<SI::Work                          >);
static_assert(verifyDimensionOrder_v<SI::YoungsModulus                 >);
