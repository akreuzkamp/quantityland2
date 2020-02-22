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

template<typename Engine_, int ...DimensionExponents>
    struct Quantity // I want Meta-classes
{
    using Engine = Engine_;
    using Scalar = numberTypeOf_t<Engine_>;

    template<typename OtherEngine>
    constexpr Quantity(const Quantity<OtherEngine, DimensionExponents...> &other);


    template<typename OtherEngine> constexpr Quantity &operator= (const Quantity<OtherEngine, DimensionExponents...> &other);
    template<typename OtherEngine, int ...OtherDimensionExponents> constexpr Quantity &operator+= (const Quantity<OtherEngine, OtherDimensionExponents...> &other);
    template<typename OtherEngine, int ...OtherDimensionExponents> constexpr Quantity &operator-= (const Quantity<OtherEngine, OtherDimensionExponents...> &other);

    template<typename OtherEngine, int ...OtherDimensionExponents> constexpr Quantity operator+ (const Quantity<OtherEngine, OtherDimensionExponents...> &other);
    template<typename OtherEngine, int ...OtherDimensionExponents> constexpr Quantity operator- (const Quantity<OtherEngine, OtherDimensionExponents...> &other);

    template<typename OtherEngine> constexpr bool operator==(const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator!=(const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator< (const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator> (const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator<=(const Quantity<OtherEngine, DimensionExponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator>=(const Quantity<OtherEngine, DimensionExponents...> &other) const;

    static constexpr Quantity fromNumericalValue(Scalar value) { return { value }; }
    constexpr Scalar numericalValue() const { return m_value; }


protected:
    constexpr Quantity<Engine, DimensionExponents...> (Scalar numericalValue) : m_value(numericalValue) {}
//     explicit constexpr operator Scalar() { return m_value; }

    Scalar m_value;
};

template<int exp, typename T>
constexpr std::decay_t<T> pow(T &&base)
{
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


inline namespace detail { //FIXME: why can't I just put this in a non-inline namespace?
    template<typename Quantity>
    constexpr auto declval() noexcept { return Quantity::fromNumericalValue(0.0); };

    template<typename T>
    auto isQuantity(const T &) -> std::false_type;
    template<typename Engine, int ...DimensionPack>
    auto isQuantity(const Quantity<Engine, DimensionPack...> &) -> std::true_type;
    template<typename T>
    constexpr bool isQuantity_v = decltype(isQuantity(std::declval<T>()))::value;

    static_assert(isQuantity_v<int> == false);
    static_assert(isQuantity_v<Quantity<void, 0, 1, 0>> == true);

    template<typename Engine1, typename Engine2, int ...DimensionPack1, int ...DimensionPack2>
    constexpr bool hasSameDimension(Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>)
    {
        return ((DimensionPack1 == DimensionPack2) && ...);
    }

    template<typename Quantity1, typename Quantity2>
    constexpr bool hasSameDimension_v = hasSameDimension(std::declval<Quantity1>(), std::declval<Quantity2>());

    template<typename NewEngine, typename OldEngine, int ...DimensionPack>
    auto changeEngine(Quantity<OldEngine, DimensionPack...>) -> Quantity<NewEngine, DimensionPack...>;

    template<typename NewEngine, typename Quantity>
    using changeEngine_t = decltype(changeEngine<NewEngine>(declval<Quantity>()));


    template<typename T, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T toNumericalValue(const T value) { return value; }
    template<typename Engine, int ...DimensionPack>
        constexpr numberTypeOf_t<Engine> toNumericalValue(const Quantity<Engine, DimensionPack...> &value) { return value.numericalValue(); }
    template<typename T, std::enable_if_t<!isQuantity_v<T>>* = nullptr>//, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const T value) { return value; }
    template<typename T, std::enable_if_t<isQuantity_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const typename T::Scalar value) { return T::fromNumericalValue(value); }

    template<typename Engine, int ...exponents>
    auto decay(Quantity<Engine, exponents...>) -> std::enable_if_t<((exponents != 0) || ...), Quantity<Engine, exponents...>>;
    template<typename Engine, int ...exponents>
    auto decay(Quantity<Engine, exponents...>) -> std::enable_if_t<((exponents == 0) && ...), numberTypeOf_t<Engine>>;

    template<typename Q>
    using decay_t = decltype(decay(declval<Q>()));

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
    template<typename Engine, int ...exponentPack, size_t ...I>
    constexpr double referenceConversionFactor_impl(const Quantity<Engine, exponentPack...> &, std::index_sequence<I...> = std::make_index_sequence<sizeof...(exponentPack)>())
    {
        return (pow<exponentPack>(toNumericalValue(std::get<I>(Engine::baseUnits))) * ...);
    }
    template<typename Engine, int ...exponentPack>
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

    template<typename Engine, int ...exponentPack, size_t ...I>
    std::string formatUnit_impl(Quantity<Engine, exponentPack...>, std::index_sequence<I...>)
    {
        auto str = ((std::string(std::get<I>(Engine::unitStrings)) + "^" + std::to_string(exponentPack) + "*") + ...);
        return str.substr(0, str.size() - 1); // removes the trailing "*"
    }
    template<typename Engine, int ...exponentPack, size_t ...I>
    std::string formatUnit(Quantity<Engine, exponentPack...> q)
    {
        return formatUnit_impl(q, std::make_index_sequence<sizeof...(exponentPack)>());
    }



    template<typename ToEngine, typename FromEngine, int ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename FromEngine::referenceEngine, ToEngine>, numberTypeOf_t<FromEngine>>
    {
        return v.numericalValue() * detail::referenceConversionFactor(v);
    }
    template<typename ToEngine, typename FromEngine, int ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, FromEngine>, numberTypeOf_t<FromEngine>>
    {
        constexpr auto ownConvFactor = detail::referenceConversionFactor(Quantity<ToEngine, DimensionPack...>::fromNumericalValue(0.0)); // the numericalValue of zero is arbitrary and won't be used
        return 1.0 / ownConvFactor * v.numericalValue();
    }
    template<typename ToEngine, typename FromEngine, int ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, typename FromEngine::referenceEngine>
                        && !std::is_same_v<typename ToEngine::referenceEngine, void>
                        && !std::is_same_v<FromEngine, ToEngine>, numberTypeOf_t<FromEngine>>
    {
        constexpr auto ownConvFactor = detail::referenceConversionFactor(v);
        constexpr auto otherConvFactor = detail::referenceConversionFactor(Quantity<ToEngine, DimensionPack...>::fromNumericalValue(0.0)); // the numericalValue of zero is arbitrary and won't be used
        return ownConvFactor / otherConvFactor * v.numericalValue();
    }
    template<typename ToEngine, int ...DimensionPack>
    constexpr numberTypeOf_t<ToEngine> convert(Quantity<ToEngine, DimensionPack...> v)
    {
        return v.numericalValue();
    }

} // namespace detail


// static_assert(std::is_same_v<mergeMul_t<Quantity<void, 0, 1, 0>, Quantity<void, 0, 1, 0>>, Quantity<void, 0, 2, 0>>);

template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr Quantity<Engine_, DimensionExponents...>::Quantity(const Quantityland2::Quantity< OtherEngine, DimensionExponents... >& other)
    : m_value { detail::convert<Engine_>(other) }
{}

template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr Quantity< Engine_, DimensionExponents... >& Quantity<Engine_, DimensionExponents...>::operator= (const Quantityland2::Quantity< OtherEngine, DimensionExponents... >& other )
{
    m_value  = detail::convert<Engine> ( other );
    return *this;
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine, int ...OtherDimensionExponents>
constexpr Quantity< Engine_, DimensionExponents... >& Quantity<Engine_, DimensionExponents...>::operator+= (const Quantityland2::Quantity< OtherEngine, OtherDimensionExponents... >& other)
{
    static_assert ( ( std::is_same_v<OtherDimensionExponents, DimensionExponents> && ... ), "Can't add quantities of differing dimension." );
    // We do the combination of broad template and static_assert to get better error messages.
    // Without the error message would be "no matching call to operator+ [...], candidates are: [List of 5 billion free operator+ overloads]"

    m_value += detail::convert<Engine> ( other );
    return *this;
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine, int ...OtherDimensionExponents>
constexpr Quantity< Engine_, DimensionExponents... >& Quantity<Engine_, DimensionExponents...>::operator-= (const Quantityland2::Quantity< OtherEngine, OtherDimensionExponents... >& other)
{
    static_assert ( ( std::is_same_v<OtherDimensionExponents, DimensionExponents> && ... ), "Can't subtract quantities of differing dimension." );
    m_value -= detail::convert<Engine> ( other );
    return *this;
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine, int ...OtherDimensionExponents>
constexpr Quantity< Engine_, DimensionExponents... > Quantity<Engine_, DimensionExponents...>::operator+ (const Quantityland2::Quantity<OtherEngine, OtherDimensionExponents...>& other)
{
    static_assert ( ( (OtherDimensionExponents == DimensionExponents) && ... ), "Can't add quantities of differing dimension." );
    return { m_value + detail::convert<Engine> ( other ) };
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine, int ...OtherDimensionExponents>
constexpr Quantity< Engine_, DimensionExponents... > Quantity<Engine_, DimensionExponents...>::operator- (const Quantityland2::Quantity<OtherEngine, OtherDimensionExponents...>& other )
{
    static_assert ( ( (OtherDimensionExponents == DimensionExponents) && ... ), "Can't subtract quantities of differing dimension." );
    return { m_value - detail::convert<Engine> ( other ) };
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionExponents...>::operator== (const Quantityland2::Quantity<OtherEngine, DimensionExponents...>& other ) const
{
    return m_value == detail::convert<Engine> ( other );
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionExponents...>::operator!= (const Quantityland2::Quantity<OtherEngine, DimensionExponents...>& other ) const
{
    return m_value != detail::convert<Engine> ( other );
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionExponents...>::operator< (const Quantityland2::Quantity<OtherEngine, DimensionExponents...>& other ) const
{
    return m_value <  detail::convert<Engine> ( other );
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionExponents...>::operator> (const Quantityland2::Quantity<OtherEngine, DimensionExponents...>& other ) const
{
    return m_value >  detail::convert<Engine> ( other );
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionExponents...>::operator<= (const Quantityland2::Quantity<OtherEngine, DimensionExponents...>& other ) const
{
    return m_value <= detail::convert<Engine> ( other );
}


template<typename Engine_, int ...DimensionExponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionExponents...>::operator>= (const Quantityland2::Quantity<OtherEngine, DimensionExponents...>& other ) const
{
    return m_value >= detail::convert<Engine> ( other );
}


template<typename Engine1, typename Engine2,int ...DimensionPack1, int ...DimensionPack2>
constexpr auto operator*(const Quantity<Engine1, DimensionPack1...> &lhs, const Quantity<Engine2, DimensionPack2...> &rhs)
    -> decay_t<Quantity<Engine1, (DimensionPack1 + DimensionPack2)...>>
{
    using Ret_t = decay_t<Quantity<Engine1, (DimensionPack1 + DimensionPack2)...>>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() * detail::convert<Engine1>(rhs));
}

template<typename Engine1, typename Engine2,int ...DimensionPack1, int ...DimensionPack2>
constexpr auto operator/(const Quantity<Engine1, DimensionPack1...> &lhs, const Quantity<Engine2, DimensionPack2...> &rhs)
    -> decay_t<Quantity<Engine1, (DimensionPack1 - DimensionPack2)...>>
{
    using Ret_t = decay_t<Quantity<Engine1, (DimensionPack1 - DimensionPack2)...>>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() / detail::convert<Engine1>(rhs));
}

template<typename Scalar, typename Engine, int ...DimensionPack>
constexpr auto operator*(const Quantity<Engine, DimensionPack...> &lhs, Scalar rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs.numericalValue() * rhs), numberTypeOf_t<Engine>>, decay_t<Quantity<Engine, DimensionPack...>>>
{
    using Ret_t = decay_t<Quantity<Engine, DimensionPack...>>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() * rhs);
}

template<typename Scalar, typename Engine, int ...DimensionPack>
constexpr auto operator*(Scalar lhs, const Quantity<Engine, DimensionPack...> &rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs * rhs.numericalValue()), numberTypeOf_t<Engine>>, decay_t<Quantity<Engine, DimensionPack...>>>
{
    using Ret_t = decay_t<Quantity<Engine, DimensionPack...>>;
    return detail::fromNumericalValue<Ret_t>(lhs * rhs.numericalValue());
}

template<typename Scalar, typename Engine, int ...DimensionPack>
constexpr auto operator/(const Quantity<Engine, DimensionPack...> &lhs, Scalar rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs.numericalValue() / rhs), numberTypeOf_t<Engine>>, decay_t<Quantity<Engine, DimensionPack...>>>
{
    using Ret_t = decay_t<Quantity<Engine, DimensionPack...>>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() / rhs);
}

template<typename Scalar, typename Engine, int ...DimensionPack>
constexpr auto operator/(Scalar lhs, const Quantity<Engine, DimensionPack...> &rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs / rhs.numericalValue()), numberTypeOf_t<Engine>>, decay_t<Quantity<Engine, -DimensionPack...>>>
{
    using Ret_t = decay_t<Quantity<Engine, -DimensionPack...>>;
    return detail::fromNumericalValue<Ret_t>(lhs / rhs.numericalValue());
}


template<int exp, typename Engine, int ...exponentPack>
auto root(const Quantity<Engine, exponentPack...> &base)
{
    static_assert(((exponentPack % exp == 0) && ...), "Extracting the root is not possible: Quantityland2 doesn't support rational exponents yet.");
    using Ret_t = Quantity<Engine, (exponentPack / exp)...>;
    if constexpr (exp == 2) {
        return detail::fromNumericalValue<Ret_t>(std::sqrt(base.numericalValue()));
    } else if constexpr (exp == 3) {
        return detail::fromNumericalValue<Ret_t>(std::cbrt(base.numericalValue()));
    } else {
        return detail::fromNumericalValue<Ret_t>(std::pow(base.numericalValue(), 1.0 / exp));
    }
}

template<typename Engine, int ...exponentPack>
std::ostream& operator<<(std::ostream& os, const Quantity<Engine, exponentPack...>& v)
{
    if constexpr ((definesUnitStrings_v<Engine>)) {
        os << v.numericalValue() << detail::formatUnit(v);
    } else if constexpr ((definesUnitStrings_v<typename Engine::referenceEngine>)) {
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
