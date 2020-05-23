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

#include <vector>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/take_while.hpp>

// We consider an ionized plasma of hydrogen gas that radiatively cools. Its temperature evolution
// is governed by the equation
//
// dT/dt = - 2/ (3kB) * nH * Λ(T)
//
// where Λ(T ) describes the cooling rate as a function of temperature, kB = 1.38 × 10−23 J/K is
// Boltzmann’s constant, and nH is the number density of hydrogen atoms. The cooling rate is a
// strong function of temperature T , which we here approximate by
//
// Λ(T) = { Λ0 * (T/T0)^α for T <= T0
//          Λ0 * (T/T0)^β for T >  T0
//
// with Λ0 = 10−35 J m3 s−1 , T0 = 20000 K, α = 10.0, and β = −0.5. We consider isochoric
// cooling of gas at density nH = 106 m−3, with an initial temperature of Tinit = 107 K.
//
// We calculate the temperature evolution T (t) by integrating equation (1) with a second-order
// Runge-Kutter scheme and a fixed timestep.



template<typename Func_t, typename X_t, typename Y_t>
auto rungeKutta2ndOrder(Func_t f, X_t x0, Y_t y0, X_t delta_x)
{
    using namespace ranges;

    auto step = [delta_x, f, x = x0, y = y0]() mutable {
        auto k1 = f(y, x);
        x += delta_x;
        auto k2 = f(y + k1 * delta_x, x);
        y += 0.5 * (k1 + k2) * delta_x;
        return std::pair {x, y};
    };

    return view::generate(step);
}

using namespace Quantityland2;
using namespace Quantityland2::SI_literals;

int main(int argc, char **argv) {
    constexpr auto T0 = 20'000_K;
    constexpr auto Lambda0 = 1e-35 * SI::units::J * SI::units::m3 * SI::units::Hz;
    constexpr auto alpha = 10.0;
    constexpr auto beta = -0.5;
    constexpr auto n_H = 1e6 / SI::units::m3;
    constexpr auto Tinit = 1e7_K;
    constexpr auto Tthreshold = 6000_K;
    constexpr auto t0 = 0.0_s;
    constexpr auto deltat = 1e10_s;
    constexpr auto maxLocalError = 50_K;
    auto Lambda = [=](auto T)
    {
        return T <= T0 ? Lambda0 * std::pow(T / T0, alpha)
                       : Lambda0 * std::pow(T / T0, beta);
    };

    auto dT_By_dt = [=](auto T, auto t)
    {
        return - 2 / (3*SI::constants::k_B) * n_H * Lambda(T);
    };
    auto fixedTimeStepData = rungeKutta2ndOrder(dT_By_dt, t0, Tinit, deltat);

    for (auto [t, T] : fixedTimeStepData | ranges::view::take_while([=](auto pair) { return pair.second > Tthreshold; })) {
        std::cout << "t == " << t << "; T(t) == " << T << std::endl;
    }
}
