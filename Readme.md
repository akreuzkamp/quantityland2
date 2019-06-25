# Quantityland2

Quantityland2 brings more type-safety to physical simulations by providing datatypes for physical
quantities and units. It provides data-types for the most common quantities and systems of units.
It is similar to boost::units, but is written in more modern C++ and provides more stuff out of
the box.

### Features

* Type checking: Can't add a mass to a length
* Quantities of arbitrary dimension: Can multiply/divide any two quantities
* No runtime overhead: Compiles to nothing but simple doubles
* Automatic unit conversion for compatible systems of units
* Support for natural units
* Fully customizable: Create your own system of units or even your own base dimensions

Missing so far:
* fractional exponents
* support for arithmetic types other than double
* non proportionate units (e.g. °C or dB)

### Install

* Download and upack the project
* Open a unix-commandline and type

```
cd path/to/quantityland2
mkdir build
cd build
cmake ..
sudo make install
```

On Linux, Quantityland2 will install its headers to `/usr/local/include/quantityland2/`. To change
that, add `-DCMAKE_INSTALL_PREFIX=/path/to/install/location/` to the `cmake` command.

### Adding Quantityland2 to you project

To use Quantityland2, first, Quantityland2 has to be found by your compiler.
If you use cmake, that's very simple. In your CMakeLists.txt, in between the lines `project(...)`
and `add_executable(...)` add the line `find_package(Quantityland2)`. Then link your executable to
`Quantityland2::quantityland2`. If your executable is named mySimulation, you can link by
adding the line `target_link_libraries(mySimulation Quantityland2::quantityland2)` somewhere below
the line `add_executable(mySimulation ...)`.

If you don't use cmake, use whatever your build system provides to add `/usr/local/include/quantityland2/`
(or where you installed Quantityland2 to) to your system include paths.

### Basic usage

Most users will simply want to use Quantityland2 for defining variables with SI units.
This is the probably simplest thing you can do with this library. In your code include
`si.h` using the line `#include <si.h>` (if you didn't use cmake, you might need to prefix it with
the path relative to the default include directory, e.g. `#include <quantityland2/si.h>`).

Next, to be able to use literals, add the line `using namespace Quantityland2::SI_literals;`. Now
you have the datatypes `SI::Length`, `SI::Mass`, `SI::Time`, etc. available. Moreover, you can use
the literals `1_m`, `2.5_kg` or `1000_ms`. Alternatively, you can use the unit-constants:
`SI::units::m`, `SI::units::kg` or `SI::units::ms`.

To summarize, here is a simple application which calculates the duration of a year in seconds:

```
# CMakeLists.txt
cmake_minimum_required(VERSION 3.0)

project(piTimesTenToTheSeven)
find_package(Quantityland2 REQUIRED)

add_executable(piTimesTenToTheSeven main.cpp)
target_link_libraries(piTimesTenToTheSeven Quantityland2::quantityland2)
```

```
// main.cpp
#include <iostream>
#include <si.h>

using namespace Quantityland2;
using namespace Quantityland2::SI_literals;
using namespace std;

auto kepler(SI::Mass m, SI::Length a) {
    return 4*pi*pi * a*a*a / ( SI::constants::G * m );
}

int main()
{
    auto sunMass = 1.9884e33 * CGS::units::g;
    auto earthMass = 5.9723e24_kg;
    auto a = 149597870700_m;

    std::cout << "A year lasts " << root<2>(kepler(sunMass + earthMass, a)) << std::endl;
    return 0;
}
```

### Customize the numeric representation

The machine precision is best for numbers near 1.0. Thus it's common in physical simulations
to not represent numbers as multiples of SI-units, but as multiples of some characteristic
quantities of the simulation. That doesn't spoil the usage of Quantityland2, quite the opposite!
In fact, this is where Quantityland2 really shines. You will be able to express your quantities
in SI-units and still have the underlying calculation be done in multiples of your characteristic
quantities.

To allow that, we need to define, what in Quantityland2 is called an Engine. The Engine is a type,
which contains all quantity-types, unit variables and constants as static members and sub types
(Quantityland2::SI is such an engine). Don't worry, creating a custom engine is not complicated at
all!

```
using namespace Quantityland2;

struct C60Quantities : public SiDimensions<C60Quantities>
{
    using referenceEngine = SI; // allow conversion from/to SI and other SI-derived engines

    template<typename T> static constexpr auto baseUnit = 1.0; // this is only there to be specialized

    using units = SI_units_template<Engine<C60Config>>; // inherit basic unit constants (like m, kg, s) from SI
    using constants = SI_constants_template<Engine<C60Config>>;  // some common constants, readily represented in C60-units :)
};
// base units in SI units, allows automatic unit conversion (ISO C++ requires specializations of member templates to happen outside the class)
template<> constexpr SI::Length          C60Quantities::baseUnit<typename base_dimensions::Length> = 0.14e-9 * SI::units::m; // average bond length in a C60-fullerene
template<> constexpr SI::Mass            C60Quantities::baseUnit<typename base_dimensions::Mass> = 1.1967e-24 * SI::units::kg; // mass of a C60-fullerene
template<> constexpr SI::Time            C60Quantities::baseUnit<typename base_dimensions::Time> = 1.0 * SI::units::s;
template<> constexpr SI::ElectricCurrent C60Quantities::baseUnit<typename base_dimensions::ElectricCurrent> = 1.0 * SI::units::A;

int main()
{
    C60Quantities::Mass m1 = 1 * C60Quantities::units::kg; // or just use SI_literals and write 1_kg, thanks to automatic unit conversion, that's the same.
    std::cout << m1 << std::endl;
}
```

