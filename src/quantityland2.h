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

#include <type_traits>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <tuple>
#include <numeric>
// #include <compare>

namespace Quantityland2 {

template<typename T, typename Enable=void> struct numberTypeOf
{
    using type = double;
};
template<typename T> struct numberTypeOf<T, std::void_t<typename T::NumberType>>
{
    using type = typename T::NumberType;
};

template<typename T> using numberTypeOf_t = typename numberTypeOf<T>::type;

struct Ratio {
    /* implicit */ constexpr Ratio(int num) : num(num), denom(1) {}
    constexpr Ratio(int num, int denom) : num(num / std::gcd(num, denom)), denom(denom / std::gcd(num, denom)) {}

    constexpr double value() const { return double { num } / denom; }

//     friend constexpr auto operator<=>(Ratio lhs, int rhs) { return (lhs.value()) <=> rhs; }
//     friend constexpr auto operator<=>(Ratio lhs, Ratio rhs) { return (lhs.value()) <=> rhs.value(); }

    friend constexpr auto operator==(Ratio lhs, int rhs) { return (lhs.value()) == rhs; }
    friend constexpr auto operator==(Ratio lhs, Ratio rhs) { return (lhs.value()) == rhs.value(); }
    friend constexpr auto operator!=(Ratio lhs, int rhs) { return (lhs.value()) != rhs; }
    friend constexpr auto operator!=(Ratio lhs, Ratio rhs) { return (lhs.value()) != rhs.value(); }

    friend constexpr auto operator<(Ratio lhs, int rhs) { return (lhs.value()) < rhs; }
    friend constexpr auto operator<(Ratio lhs, Ratio rhs) { return (lhs.value()) < rhs.value(); }
    friend constexpr auto operator>(Ratio lhs, int rhs) { return (lhs.value()) > rhs; }
    friend constexpr auto operator>(Ratio lhs, Ratio rhs) { return (lhs.value()) > rhs.value(); }


    Ratio constexpr friend operator+(Ratio lhs, Ratio rhs) { auto denom = std::lcm(lhs.denom, rhs.denom);  return Ratio { (denom / lhs.denom) * lhs.num + (denom / rhs.denom) * rhs.num, denom }; }
    Ratio constexpr friend operator-(Ratio lhs, Ratio rhs) { auto denom = std::lcm(lhs.denom, rhs.denom);  return Ratio { (denom / lhs.denom) * lhs.num - (denom / rhs.denom) * rhs.num, denom }; }
    Ratio constexpr friend operator-(Ratio r) { return Ratio { -r.num, r.denom }; }
    Ratio constexpr friend operator*(Ratio lhs, Ratio rhs) { return Ratio { lhs.num * rhs.num, lhs.denom * rhs.denom }; }
    Ratio constexpr friend operator/(Ratio lhs, Ratio rhs) { return Ratio { lhs.num * rhs.denom, lhs.denom * rhs.num }; }


    const int num;
    const int denom;
};

}

namespace std {
    std::string to_string(Quantityland2::Ratio r)
    {
        return to_string(r.num) + "/" + to_string(r.denom);
    }

}

namespace Quantityland2 {


template<typename Engine_, Ratio ...DimensionExponents>
class Quantity;


namespace detail {
    template<typename Quantity>
    constexpr Quantity &declval() noexcept;// { return *reinterpret_cast<Quantity*>(0); }

    template<typename T>
    auto isQuantity(T &) -> std::false_type;
    template<typename Engine, Ratio ...DimensionPack>
    auto isQuantity(Quantity<Engine, DimensionPack...> &) -> std::true_type;
    template<typename T>
    constexpr bool isQuantity_v = decltype(isQuantity(declval<T>()))::value;

    template<typename Engine, Ratio ...exponents>
    auto decay(Quantity<Engine, exponents...>&) -> std::enable_if_t<((exponents != 0) || ...), Quantity<Engine, exponents...>>;
    template<typename Engine, Ratio ...exponents>
    auto decay(Quantity<Engine, exponents...>&) -> std::enable_if_t<((exponents == 0) && ...), numberTypeOf_t<Engine>>;

    template<typename Q>
    using decay_t = decltype(decay(declval<Q>()));



    template<typename ToEngine, typename FromEngine, Ratio ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename FromEngine::referenceEngine, ToEngine>, numberTypeOf_t<FromEngine>>;
    template<typename ToEngine, typename FromEngine, Ratio ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, FromEngine>, numberTypeOf_t<FromEngine>>;
    template<typename ToEngine, typename FromEngine, Ratio ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, typename FromEngine::referenceEngine>
                        && !std::is_same_v<typename ToEngine::referenceEngine, void>
                        && !std::is_same_v<FromEngine, ToEngine>, numberTypeOf_t<FromEngine>>;
    template<typename ToEngine, Ratio ...DimensionPack>
    constexpr numberTypeOf_t<ToEngine> convert(Quantity<ToEngine, DimensionPack...> v);




    template<typename T, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T toNumericalValue(const T value);
    template<typename Engine, Ratio ...DimensionPack>
        constexpr numberTypeOf_t<Engine> toNumericalValue(const Quantity<Engine, DimensionPack...> &value);
    template<typename T, std::enable_if_t<!isQuantity_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const T value);
    template<typename T, std::enable_if_t<isQuantity_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const typename T::Scalar value);
}

template<typename Engine_, Ratio ...DimensionExponents>
    class Quantity // I want Meta-classes
{
public:
    using Engine = Engine_;
    using Scalar = numberTypeOf_t<Engine_>;

//     template<typename OtherEngine>
//     constexpr Quantity(const Quantity<OtherEngine, DimensionExponents...> &other);

/*
    template<typename OtherEngine> constexpr Quantity &operator= (const Quantity<OtherEngine, DimensionExponents...> &other);
    template<typename OtherEngine, Ratio ...OtherDimensionExponents> constexpr Quantity &operator+= (const Quantity<OtherEngine, OtherDimensionExponents...> &other);
    template<typename OtherEngine, Ratio ...OtherDimensionExponents> constexpr Quantity &operator-= (const Quantity<OtherEngine, OtherDimensionExponents...> &other);

    template<typename OtherEngine, Ratio ...OtherDimensionExponents> constexpr Quantity operator+ (const Quantity<OtherEngine, OtherDimensionExponents...> &other);
    template<typename OtherEngine, Ratio ...OtherDimensionExponents> constexpr Quantity operator- (const Quantity<OtherEngine, OtherDimensionExponents...> &other);

    template<typename OtherEngine> constexpr bool operator==(const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator!=(const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator< (const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator> (const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator<=(const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator>=(const Quantity<OtherEngine, DimensionExponents...> &other) const;*/


    static constexpr Quantity fromNumericalValue(Scalar value) { return { value }; }
    constexpr Scalar numericalValue() const { return m_value; }


    template<typename OtherEngine>
    constexpr Quantity(const Quantity< OtherEngine, DimensionExponents... >& rhs)
        : m_value { detail::convert<Engine_>(rhs) }
    {}

    template<typename OtherEngine>
    constexpr Quantity< Engine_, DimensionExponents... > &operator= (const Quantity< OtherEngine, DimensionExponents... >& rhs)
    {
        m_value  = detail::convert<Engine> ( rhs );
        return *this;
    }


    template<typename OtherEngine, Ratio ...OtherDimensionExponents>
    constexpr Quantity< Engine_, DimensionExponents... >& operator+= (const Quantity< OtherEngine, OtherDimensionExponents... >& rhs)
    {
        static_assert ( ( std::is_same_v<OtherDimensionExponents, DimensionExponents> && ... ), "Can't add quantities of differing dimension." );
        // We do the combination of broad template and static_assert to get better error messages.
        // Without the error message would be "no matching call to operator+ [...], candidates are: [List of 5 billion free operator+ overloads]"

        m_value += detail::convert<Engine> ( rhs );
        return *this;
    }


    template<typename OtherEngine, Ratio ...OtherDimensionExponents>
    constexpr Quantity< Engine_, DimensionExponents... >& operator-= (const Quantity< OtherEngine, OtherDimensionExponents... >& rhs)
    {
        static_assert ( ( std::is_same_v<OtherDimensionExponents, DimensionExponents> && ... ), "Can't subtract quantities of differing dimension." );
        m_value -= detail::convert<Engine> ( rhs );
        return *this;
    }


    template<typename OtherEngine, Ratio ...OtherDimensionExponents>
    constexpr friend Quantity< Engine_, DimensionExponents... > operator+ (const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, OtherDimensionExponents...>& rhs)
    {
        static_assert ( ( (OtherDimensionExponents == DimensionExponents) && ... ), "Can't add quantities of differing dimension." );
        return { lhs.m_value + detail::convert<Engine> ( rhs ) };
    }


    template<typename OtherEngine, Ratio ...OtherDimensionExponents>
    constexpr friend Quantity< Engine_, DimensionExponents... > operator- (const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, OtherDimensionExponents...>& rhs)
    {
        static_assert ( ( (OtherDimensionExponents == DimensionExponents) && ... ), "Can't subtract quantities of differing dimension." );
        return { lhs.m_value - detail::convert<Engine> ( rhs ) };
    }


    template<typename OtherEngine>
    constexpr friend bool operator==(const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, DimensionExponents...>& rhs )
    {
        return lhs.m_value == detail::convert<Engine> ( rhs );
    }


    template<typename OtherEngine>
    constexpr friend bool operator!=(const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, DimensionExponents...>& rhs )
    {
        return lhs.m_value != detail::convert<Engine> ( rhs );
    }


    template<typename OtherEngine>
    constexpr friend bool operator<(const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, DimensionExponents...>& rhs )
    {
        return lhs.m_value <  detail::convert<Engine> ( rhs );
    }


    template<typename OtherEngine>
    constexpr friend bool operator>(const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, DimensionExponents...>& rhs )
    {
        return lhs.m_value >  detail::convert<Engine> ( rhs );
    }


    template<typename OtherEngine>
    constexpr friend bool operator<=(const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, DimensionExponents...>& rhs )
    {
        return lhs.m_value <= detail::convert<Engine> ( rhs );
    }


    template<typename OtherEngine>
    constexpr friend bool operator>=(const Quantity<Engine_, DimensionExponents...> &lhs, const Quantity<OtherEngine, DimensionExponents...>& rhs )
    {
        return lhs.m_value >= detail::convert<Engine> ( rhs );
    }


    template<typename Engine2, Ratio ...DimensionPack2>
    constexpr friend auto operator*(const Quantity<Engine, DimensionExponents...> &lhs, const Quantity<Engine2, DimensionPack2...> &rhs)
        -> detail::decay_t<Quantity<Engine, (DimensionExponents + DimensionPack2)...>>
    {
        using Ret_t = detail::decay_t<Quantity<Engine, (DimensionExponents + DimensionPack2)...>>;
        return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() * detail::convert<Engine>(rhs));
    }

    template<typename Engine2, Ratio ...DimensionPack2>
    constexpr friend auto operator/(const Quantity<Engine, DimensionExponents...> &lhs, const Quantity<Engine2, DimensionPack2...> &rhs)
        -> detail::decay_t<Quantity<Engine, (DimensionExponents - DimensionPack2)...>>
    {
        using Ret_t = detail::decay_t<Quantity<Engine, (DimensionExponents - DimensionPack2)...>>;
        return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() / detail::convert<Engine>(rhs));
    }

    template<typename Scalar>
    constexpr friend auto operator*(const Quantity<Engine, DimensionExponents...> &lhs, Scalar rhs)
        -> std::enable_if_t<!detail::isQuantity_v<Scalar>, Quantity>
    {
        static_assert(std::is_convertible_v<decltype(lhs.numericalValue() * rhs), numberTypeOf_t<Engine>>, "Scalar type incompatible with Quantity's value type.");
        return Quantity::fromNumericalValue(lhs.numericalValue() * rhs);
    }

    template<typename Scalar>
    constexpr friend auto operator*(Scalar lhs, const Quantity<Engine, DimensionExponents...> &rhs)
        -> std::enable_if_t<!detail::isQuantity_v<Scalar>, Quantity>
    {
        static_assert(std::is_convertible_v<decltype(lhs * rhs.numericalValue()), numberTypeOf_t<Engine>>, "Scalar type incompatible with Quantity's value type.");
        return Quantity::fromNumericalValue(lhs * rhs.numericalValue());
    }

    template<typename Scalar>
    constexpr friend auto operator/(const Quantity<Engine, DimensionExponents...> &lhs, Scalar rhs)
        -> std::enable_if_t<!detail::isQuantity_v<Scalar>, Quantity>
    {
        static_assert(std::is_convertible_v<decltype(lhs.numericalValue() / rhs), numberTypeOf_t<Engine>>, "Scalar type incompatible with Quantity's value type.");
        return Quantity::fromNumericalValue(lhs.numericalValue() / rhs);
    }

    template<typename Scalar>
    constexpr friend auto operator/(Scalar lhs, const Quantity<Engine, DimensionExponents...> &rhs)
        -> std::enable_if_t<!detail::isQuantity_v<Scalar>, Quantity<Engine, -DimensionExponents...>>
    {
        static_assert(std::is_convertible_v<decltype(lhs / rhs.numericalValue()), numberTypeOf_t<Engine>>, "Scalar type incompatible with Quantity's value type.");
        using Ret_t = Quantity<Engine, -DimensionExponents...>;
        return Ret_t::fromNumericalValue(lhs / rhs.numericalValue());
    }


protected:
    constexpr /*explicit*/ Quantity(Scalar numericalValue) : m_value(numericalValue) {}
//     explicit constexpr operator Scalar() { return m_value; }

    Scalar m_value;
};

template<Ratio exp_, typename T>
constexpr auto pow(T &&base)
    -> std::enable_if_t<exp_.denom == 1, std::decay_t<T>>
{
    constexpr int exp = exp_.num;
    if constexpr (exp < 0) {
        return 1.0 / pow<-exp>(base);
    } else if constexpr (exp == 0) {
        return 1.0;
    } else if constexpr (exp == 2) {
        return base * base;
    } else if constexpr (exp % 2 == 0) {
        constexpr auto tmp = pow<exp / 2>(base); // FIXME why can't this be constexpr?
        return tmp * tmp;
    } else {
        return base * pow<exp - 1>(base);
    }
}


template<Ratio exp, typename Engine, Ratio ...exponentPack>
auto pow(const Quantity<Engine, exponentPack...> &base)
    -> std::enable_if_t<exp.denom != 1, detail::decay_t<Quantity<Engine, (exponentPack * exp)...>>>
{
    using Ret_t = detail::decay_t<Quantity<Engine, (exponentPack * exp)...>>;
    return detail::fromNumericalValue<Ret_t>(std::pow(base.numericalValue(), exp));
}

namespace detail { //FIXME: why can't I just put this in a non-inline namespace?
    static_assert(isQuantity_v<int> == false);
//     static_assert(isQuantity_v<Quantity<void, 0, 1, 0>> == true);

    template<typename Quantity>
    constexpr Quantity &declval() noexcept { auto q = Quantity::fromNumericalValue(0.0); return q; }

    template<typename Engine1, typename Engine2, Ratio ...DimensionPack1, Ratio ...DimensionPack2>
    constexpr bool hasSameDimension(Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>)
    {
        return ((DimensionPack1 == DimensionPack2) && ...);
    }

    template<typename Quantity1, typename Quantity2>
    constexpr bool hasSameDimension_v = hasSameDimension(std::declval<Quantity1>(), std::declval<Quantity2>());

    template<typename NewEngine, typename OldEngine, Ratio ...DimensionPack>
    auto changeEngine(Quantity<OldEngine, DimensionPack...>) -> Quantity<NewEngine, DimensionPack...>;

    template<typename NewEngine, typename Quantity>
    using changeEngine_t = decltype(changeEngine<NewEngine>(declval<Quantity>()));


    template<typename T, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T toNumericalValue(const T value) { return value; }
    template<typename Engine, Ratio ...DimensionPack>
        constexpr numberTypeOf_t<Engine> toNumericalValue(const Quantity<Engine, DimensionPack...> &value) { return value.numericalValue(); }
    template<typename T, std::enable_if_t<!isQuantity_v<T>>* = nullptr>//, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const T value) { return value; }
    template<typename T, std::enable_if_t<isQuantity_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const typename T::Scalar value) { return T::fromNumericalValue(value); }
/*
    template<typename Engine, Ratio ...exponents>
    auto decay(Quantity<Engine, exponents...>) -> std::enable_if_t<((exponents != 0) || ...), Quantity<Engine, exponents...>>;
    template<typename Engine, Ratio ...exponents>
    auto decay(Quantity<Engine, exponents...>) -> std::enable_if_t<((exponents == 0) && ...), numberTypeOf_t<Engine>>;

    template<typename Q>
    using decay_t = decltype(decay(declval<Q>()));*/

    /**
    * Given
    * `auto v = Quantity&lt;Engine, exponentPack...&gt;::fromNumericalValue(1.0);` and
    * `Quantity&lt;typename Engine::referenceEngine, exponentPack...&gt; r = v;`
    * the referenceConversionFactor is `v.numericalValue()`.
    *
    * In other words, the referenceConversionFactor is the factor needed for conversion of Engine to Engine::referenceEngine.
    *
    * Example: The cgs system has the SI system as its reference. The base unit of cgs is centimeter,
    * the base unit of SI is meter. Thus `referenceConversionFactor(cgs::Length{}) == 1cm/1m == 0.01`.
    */
    template<typename Engine, Ratio ...exponentPack, size_t ...I>
    constexpr auto referenceConversionFactor_impl(const Quantity<Engine, exponentPack...> &, std::index_sequence<I...> = std::make_index_sequence<sizeof...(exponentPack)>())
        -> std::enable_if_t<((exponentPack.denom == 1) && ...), double>
    {
        return (pow<exponentPack.num>(toNumericalValue(std::get<I>(Engine::baseUnits))) * ...);
    }
    template<typename Engine, Ratio ...exponentPack, size_t ...I>
    auto referenceConversionFactor_impl(const Quantity<Engine, exponentPack...> &, std::index_sequence<I...> = std::make_index_sequence<sizeof...(exponentPack)>())
        -> std::enable_if_t<((exponentPack.denom != 1) || ...), double>
    {
        return (std::pow(toNumericalValue(std::get<I>(Engine::baseUnits)), exponentPack.value()) * ...);
    }
    template<typename Engine, Ratio ...exponentPack>
    constexpr double referenceConversionFactor(const Quantity<Engine, exponentPack...> &q)
    {
        return referenceConversionFactor_impl(q, std::make_index_sequence<sizeof...(exponentPack)>());
    }

    template<typename ...Args>
    auto definesUnitStrings(Args ...args) -> std::false_type;
    template<typename Engine>
    auto definesUnitStrings(const Engine &) -> std::enable_if_t< (Engine::unitStrings, true) , std::true_type>;
    template<typename Engine>
    constexpr bool definesUnitStrings_v = decltype(definesUnitStrings(std::declval<Engine>()))::value;

    template<typename Engine, Ratio ...exponentPack, size_t ...I>
    std::string formatUnit_impl(Quantity<Engine, exponentPack...>, std::index_sequence<I...>)
    {
        auto str = ((std::string(std::get<I>(Engine::unitStrings)) + "^" + std::to_string(exponentPack) + "*") + ...);
        return str.substr(0, str.size() - 1); // removes the trailing "*"
    }
    template<typename Engine, Ratio ...exponentPack, size_t ...I>
    std::string formatUnit(Quantity<Engine, exponentPack...> q)
    {
        return formatUnit_impl(q, std::make_index_sequence<sizeof...(exponentPack)>());
    }



    template<typename ToEngine, typename FromEngine, Ratio ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename FromEngine::referenceEngine, ToEngine>, numberTypeOf_t<FromEngine>>
    {
        return v.numericalValue() * detail::referenceConversionFactor(v);
    }
    template<typename ToEngine, typename FromEngine, Ratio ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, FromEngine>, numberTypeOf_t<FromEngine>>
    {
        constexpr auto ownConvFactor = detail::referenceConversionFactor(Quantity<ToEngine, DimensionPack...>::fromNumericalValue(0.0)); // the numericalValue of zero is arbitrary and won't be used
        return 1.0 / ownConvFactor * v.numericalValue();
    }
    template<typename ToEngine, typename FromEngine, Ratio ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, typename FromEngine::referenceEngine>
                        && !std::is_same_v<typename ToEngine::referenceEngine, void>
                        && !std::is_same_v<FromEngine, ToEngine>, numberTypeOf_t<FromEngine>>
    {
        constexpr auto ownConvFactor = detail::referenceConversionFactor(v);
        constexpr auto rhsConvFactor = detail::referenceConversionFactor(Quantity<ToEngine, DimensionPack...>::fromNumericalValue(0.0)); // the numericalValue of zero is arbitrary and won't be used
        return ownConvFactor / rhsConvFactor * v.numericalValue();
    }
    template<typename ToEngine, Ratio ...DimensionPack>
    constexpr numberTypeOf_t<ToEngine> convert(Quantity<ToEngine, DimensionPack...> v)
    {
        return v.numericalValue();
    }

} // namespace detail


// static_assert(std::is_same_v<mergeMul_t<Quantity<void, 0, 1, 0>, Quantity<void, 0, 1, 0>>, Quantity<void, 0, 2, 0>>);


template<Ratio exp, typename Engine, Ratio ...exponentPack>
auto root(const Quantity<Engine, exponentPack...> &base)
{
//     static_assert(((exponentPack % exp == 0) && ...), "Extracting the root is not possible: Quantityland2 doesn't support rational exponents yet.");
    using Ret_t = detail::decay_t<Quantity<Engine, (exponentPack / exp)...>>;
    if constexpr (exp == 2) {
        return detail::fromNumericalValue<Ret_t>(std::sqrt(base.numericalValue()));
    } else if constexpr (exp == 3) {
        return detail::fromNumericalValue<Ret_t>(std::cbrt(base.numericalValue()));
    } else {
        return detail::fromNumericalValue<Ret_t>(std::pow(base.numericalValue(), 1.0 / exp));
    }
}

template<typename Engine, Ratio ...exponentPack>
std::ostream& operator<<(std::ostream& os, const Quantity<Engine, exponentPack...>& v)
{
    if constexpr ((detail::definesUnitStrings_v<Engine>)) {
        os << v.numericalValue() << detail::formatUnit(v);
    } else if constexpr ((detail::definesUnitStrings_v<typename Engine::referenceEngine>)) {
        Quantity<typename Engine::referenceEngine, exponentPack...> convertedV = v;
        os << convertedV.numericalValue() << detail::formatUnit(convertedV);
    } else {
        os << v.numericalValue() << " arb. unit";
    }
    return os;
}

template<typename Engine>
struct SystemOfQuantities : public Engine, public Engine::SystemOfDimensions
{
};

}
