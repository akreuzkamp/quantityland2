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

namespace Quantityland2 {

namespace base_dimensions {

struct Length
{
    static constexpr uint64_t baseDimensionId = 1;
};

struct Mass
{
    static constexpr uint64_t baseDimensionId = 2;
};

struct Time
{
    static constexpr uint64_t baseDimensionId = 3;
};

struct ElectricCurrent
{
    static constexpr uint64_t baseDimensionId = 4;
};

struct Temperature
{
    static constexpr uint64_t baseDimensionId = 5;
};

struct AmountOfSubstance
{
    static constexpr uint64_t baseDimensionId = 6;
};

struct Luminosity
{
    static constexpr uint64_t baseDimensionId = 7;
};

// template<typename T, uint64_t id = 0>
// struct isBaseDimension : public std::false_type
// {};
// template<typename T>
// struct isBaseDimension<T, T::baseDimensionId> : public std::true_type
// {};

} // namespace base_dimensions



template<typename BaseDimension_t, int Exponent>
struct DimensionComponent
{
    using Dimension = BaseDimension_t;
    static constexpr int exponent = Exponent;
};

template<typename Engine_, typename ...DimensionComponents>
    struct Quantity // I want Meta-classes
{
    using Engine = Engine_;
    using Scalar = double;

    template<typename OtherEngine>
    constexpr Quantity(const Quantity<OtherEngine, DimensionComponents...> &other);


    template<typename OtherEngine> constexpr Quantity &operator= (const Quantity<OtherEngine, DimensionComponents...> &other);
    template<typename OtherEngine, typename ...OtherDimensionComponents> constexpr Quantity &operator+= (const Quantity<OtherEngine, OtherDimensionComponents...> &other);
    template<typename OtherEngine, typename ...OtherDimensionComponents> constexpr Quantity &operator-= (const Quantity<OtherEngine, OtherDimensionComponents...> &other);

    template<typename OtherEngine, typename ...OtherDimensionComponents> constexpr Quantity operator+ (const Quantity<OtherEngine, OtherDimensionComponents...> &other);
    template<typename OtherEngine, typename ...OtherDimensionComponents> constexpr Quantity operator- (const Quantity<OtherEngine, OtherDimensionComponents...> &other);

    template<typename OtherEngine> constexpr bool operator==(const Quantity<OtherEngine, DimensionComponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator!=(const Quantity<OtherEngine, DimensionComponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator< (const Quantity<OtherEngine, DimensionComponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator> (const Quantity<OtherEngine, DimensionComponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator<=(const Quantity<OtherEngine, DimensionComponents...> &other) const;
    template<typename OtherEngine> constexpr bool operator>=(const Quantity<OtherEngine, DimensionComponents...> &other) const;

    static constexpr Quantity fromNumericalValue(Scalar value) { return { value }; }
    constexpr Scalar numericalValue() const { return m_value; }


protected:
    constexpr Quantity<Engine, DimensionComponents...> (Scalar numericalValue) : m_value(numericalValue) {}
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

    template<typename T>
    auto isQuantity(const T &) -> std::false_type;
    template<typename Engine, typename ...DimensionPack>
    auto isQuantity(const Quantity<Engine, DimensionPack...> &) -> std::true_type;
    template<typename T>
    constexpr bool isQuantity_v = decltype(isQuantity(std::declval<T>()))::value;

    static_assert(isQuantity_v<int> == false);
    static_assert(isQuantity_v<Quantity<void, DimensionComponent<base_dimensions::Length, 2>>> == true);

    template<typename Engine1, typename Engine2, typename ...DimensionPack1, typename ...DimensionPack2>
    auto hasSameDimension(Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>) -> std::conjunction<std::is_same<DimensionPack1, DimensionPack2>...>;

    template<typename Quantity1, typename Quantity2>
    constexpr bool hasSameDimension_v = decltype(hasSameDimension(std::declval<Quantity1>(), std::declval<Quantity2>()))::value;

    template<typename NewEngine, typename OldEngine, typename ...DimensionPack>
    auto changeEngine(Quantity<OldEngine, DimensionPack...>) -> Quantity<NewEngine, DimensionPack...>;

    template<typename NewEngine, typename Quantity>
    using changeEngine_t = decltype(changeEngine<NewEngine>(std::declval<Quantity>()));

    template<typename Engine, typename Head, typename ...Tail>
    auto prepend(Head, Quantity<Engine, Tail...>) -> Quantity<Engine, Head, Tail...>;

    template<typename Engine, typename Head>
    auto prepend(Head, double) -> Quantity<Engine, Head>;

    template<typename Engine, typename Head, typename Quantity>
    using prepend_t = decltype(prepend<Engine>(std::declval<Head>(), std::declval<Quantity>()));

    enum Op {
        Plus,
        Minus
    };

    constexpr int applyOp(Op o, int lhs, int rhs) { return o == Plus ? lhs + rhs : lhs - rhs; }

    // template<typename Q1, typename Q2>
    // struct Merge
    // {
    //     using type = decltype(mergeStep<Plus>(std::declval<T1>(), std::declval<T2>()));
    // };

    // Merge empty (i.e. dimensionless) quantities
    template<Op op, typename Engine1, typename Engine2>
    auto mergeStep(const Quantity<Engine1> &, const Quantity<Engine2> &)
        -> double;

    // Merge with empty right-hand-side
    template<Op op, typename Engine1, typename Engine2, typename DimensionPack1Head, typename ...DimensionPack1Tail>
    auto mergeStep(const Quantity<Engine1, DimensionPack1Head, DimensionPack1Tail...> &, const Quantity<Engine2> &)
        -> Quantity<Engine1, DimensionComponent<typename DimensionPack1Head::Dimension, applyOp(op, DimensionPack1Head::exponent, 0)>,
                             DimensionComponent<typename DimensionPack1Tail::Dimension, applyOp(op, DimensionPack1Tail::exponent, 0)>...>;


    // Merge with empty left-hand-side
    template<Op op, typename Engine1, typename Engine2, typename DimensionPack2Head, typename ...DimensionPack2Tail>
    auto mergeStep(const Quantity<Engine1> &, const Quantity<Engine2, DimensionPack2Head, DimensionPack2Tail...> &)
        -> Quantity<Engine1, DimensionComponent<typename DimensionPack2Head::Dimension, applyOp(op, 0, DimensionPack2Head::exponent)>,
                             DimensionComponent<typename DimensionPack2Tail::Dimension, applyOp(op, 0, DimensionPack2Tail::exponent)>...>;

    // Merge: same dimension, sum/difference non-zero
    template<Op op, typename Engine1, typename Engine2, typename DimensionPack1Head, typename ...DimensionPack1Tail, typename DimensionPack2Head, typename ...DimensionPack2Tail>
    auto mergeStep(const Quantity<Engine1, DimensionPack1Head, DimensionPack1Tail...> &, const Quantity<Engine2, DimensionPack2Head, DimensionPack2Tail...> &)
        -> std::enable_if_t<DimensionPack1Head::Dimension::baseDimensionId == DimensionPack2Head::Dimension::baseDimensionId
                            && applyOp(op, DimensionPack1Head::exponent, DimensionPack2Head::exponent) != 0,
                    prepend_t<Engine1, DimensionComponent<typename DimensionPack1Head::Dimension, applyOp(op, DimensionPack1Head::exponent, DimensionPack2Head::exponent)>,
                            decltype(mergeStep<op>(std::declval<Quantity<Engine1, DimensionPack1Tail...>>(), std::declval<Quantity<Engine2, DimensionPack2Tail...>>()))
                    >>;

    // Merge: same dimension, sum/difference == 0 -> remove dimension
    template<Op op, typename Engine1, typename Engine2, typename DimensionPack1Head, typename ...DimensionPack1Tail, typename DimensionPack2Head, typename ...DimensionPack2Tail>
    auto mergeStep(const Quantity<Engine1, DimensionPack1Head, DimensionPack1Tail...> &, const Quantity<Engine2, DimensionPack2Head, DimensionPack2Tail...> &)
        -> std::enable_if_t<DimensionPack1Head::Dimension::baseDimensionId == DimensionPack2Head::Dimension::baseDimensionId
                            && applyOp(op, DimensionPack1Head::exponent, DimensionPack2Head::exponent) == 0,
                decltype(mergeStep<op>(std::declval<Quantity<Engine1, DimensionPack1Tail...>>(), std::declval<Quantity<Engine2, DimensionPack2Tail...>>()))
        >;

    // Merge: lhs dimension < rhs dimension -> insert lhs dimension into result
    template<Op op, typename Engine1, typename Engine2, typename DimensionPack1Head, typename ...DimensionPack1Tail, typename DimensionPack2Head, typename ...DimensionPack2Tail>
    auto mergeStep(const Quantity<Engine1, DimensionPack1Head, DimensionPack1Tail...> &, const Quantity<Engine2, DimensionPack2Head, DimensionPack2Tail...> &)
        -> std::enable_if_t<(DimensionPack1Head::Dimension::baseDimensionId < DimensionPack2Head::Dimension::baseDimensionId),
                prepend_t<Engine1, DimensionComponent<typename DimensionPack1Head::Dimension, applyOp(op, DimensionPack1Head::exponent, 0)>,
                    decltype(mergeStep<op>(std::declval<Quantity<Engine1, DimensionPack1Tail...>>(), std::declval<Quantity<Engine2, DimensionPack2Head, DimensionPack2Tail...>>()))
                    >>;


    // Merge: lhs dimension > rhs dimension -> insert rhs dimension into result
    template<Op op, typename Engine1, typename Engine2, typename DimensionPack1Head, typename ...DimensionPack1Tail, typename DimensionPack2Head, typename ...DimensionPack2Tail>
    auto mergeStep(const Quantity<Engine1, DimensionPack1Head, DimensionPack1Tail...> &, const Quantity<Engine2, DimensionPack2Head, DimensionPack2Tail...> &)
        -> std::enable_if_t<(DimensionPack1Head::Dimension::baseDimensionId > DimensionPack2Head::Dimension::baseDimensionId),
                    prepend_t<Engine1, DimensionComponent<typename DimensionPack2Head::Dimension, applyOp(op, 0, DimensionPack2Head::exponent)>,
                    decltype(mergeStep<op>(std::declval<Quantity<Engine1, DimensionPack1Head, DimensionPack1Tail...>>(), std::declval<Quantity<Engine2, DimensionPack2Tail...>>()))
                    >>;

    template<typename Engine, typename ...DimensionPack, int ...exponentPack>
    auto invertDimension(const Quantity<Engine, DimensionComponent<DimensionPack, exponentPack>...> &)
        -> Quantity<Engine, DimensionComponent<DimensionPack, -exponentPack>...>;

    template<typename T1, typename T2>
    using mergeMul_t = decltype(mergeStep<Plus>(std::declval<T1>(), std::declval<T2>()));

    template<typename T1, typename T2>
    using mergeDiv_t = decltype(mergeStep<Minus>(std::declval<T1>(), std::declval<T2>()));

    template<typename T>
    using invert_t = decltype(invertDimension(std::declval<T>()));

    template<typename Engine, typename DimensionPackHead1, typename DimensionPackHead2, typename ...DimensionPackTail>
    static constexpr bool verifyDimensionOrder(const Quantity<Engine, DimensionPackHead1, DimensionPackHead2, DimensionPackTail...> *)
    {
        return (DimensionPackHead1::Dimension::baseDimensionId < DimensionPackHead2::Dimension::baseDimensionId)
                && verifyDimensionOrder(static_cast<Quantity<Engine, DimensionPackHead2, DimensionPackTail...>*>(nullptr));
    }
    template<typename Engine, typename DimensionPackHead1>
    static constexpr bool verifyDimensionOrder(const Quantity<Engine, DimensionPackHead1> *)
    {
        return true;
    }
    template<typename Quantity>
    static constexpr bool verifyDimensionOrder_v = verifyDimensionOrder(static_cast<Quantity*>(nullptr));

    /**
    * Given
    * `auto v = Quantity&lt;Engine, DimensionComponent&lt;BaseDimensionPack, exponentPack>...&gt;&gt;::fromNumericalValue(1.0);` and
    * `Quantity&lt;typename Engine::referenceEngine, DimensionComponent&lt;BaseDimensionPack, exponentPack>...&gt;&gt; r = v;`
    * the referenceConversionFactor is `v.numericalValue()`.
    *
    * In other words, the referenceConversionFactor is the factor needed for conversion of Engine to Engine::referenceEngine.
    *
    * Example: The cgs system has the SI system as its reference. The base unit of cgs is centimeter,
    * the base unit of SI is meter. Thus `referenceConversionFactor(cgs::Length{}) == 1cm/1m == 0.01`.
    */
    template<typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
    constexpr double referenceConversionFactor(const Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...> &)
    {
        return (pow<exponentPack>(Engine::template baseUnit<BaseDimensionPack>.numericalValue()) * ...);
    }

    template<typename BaseUnit, typename ...Args>
    // struct definesUnitString : std::false_type;
    auto definesUnitString(Args ...args) -> std::false_type;
    template<typename BaseUnit, typename Engine>
    auto definesUnitString(const Engine &) -> std::enable_if_t< (Engine::template unitString<BaseUnit>, true) , std::true_type>;
    template<typename BaseUnit, typename Engine>
    constexpr bool definesUnitString_v = decltype(definesUnitString<BaseUnit>(std::declval<Engine>()))::value;

    template<typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
    std::string formatUnit(Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...>)
    {
        auto str = ((std::string(Engine::template unitString<BaseDimensionPack>) + "^" + std::to_string(exponentPack) + "*") + ...);
        return str.substr(0, str.size() - 1); // removes the trailing "*"
    }


    template<typename T, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T toNumericalValue(const T value) { return value; }
    template<typename Engine, typename ...DimensionPack>
        constexpr double toNumericalValue(const Quantity<Engine, DimensionPack...> &value) { return value.numericalValue(); }
    template<typename T, std::enable_if_t<std::is_arithmetic_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const T value) { return value; }
    template<typename T, std::enable_if_t<isQuantity_v<T>>* = nullptr>
        constexpr T fromNumericalValue(const typename T::Scalar value) { return T::fromNumericalValue(value); }



    template<typename ToEngine, typename FromEngine, typename ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename FromEngine::referenceEngine, ToEngine>, double>
    {
        return v.numericalValue() * detail::referenceConversionFactor(v);
    }
    template<typename ToEngine, typename FromEngine, typename ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, FromEngine>, double>
    {
        constexpr auto ownConvFactor = detail::referenceConversionFactor(Quantity<ToEngine, DimensionPack...>::fromNumericalValue(0.0)); // the numericalValue of zero is arbitrary and won't be used
        return 1.0 / ownConvFactor * v.numericalValue();
    }
    template<typename ToEngine, typename FromEngine, typename ...DimensionPack>
    constexpr auto convert(Quantity<FromEngine, DimensionPack...> v)
        -> std::enable_if_t<std::is_same_v<typename ToEngine::referenceEngine, typename FromEngine::referenceEngine>
                        && !std::is_same_v<typename ToEngine::referenceEngine, void>
                        && !std::is_same_v<FromEngine, ToEngine>, double>
    {
        constexpr auto ownConvFactor = detail::referenceConversionFactor(v);
        constexpr auto otherConvFactor = detail::referenceConversionFactor(Quantity<ToEngine, DimensionPack...>::fromNumericalValue(0.0)); // the numericalValue of zero is arbitrary and won't be used
        return ownConvFactor / otherConvFactor * v.numericalValue();
    }
    template<typename ToEngine, typename ...DimensionPack>
    constexpr double convert(Quantity<ToEngine, DimensionPack...> v)
    {
        return v.numericalValue();
    }

} // namespace detail


// static_assert(std::is_same_v<mergeMul_t<Quantity<void, DimensionComponent<Length, 1>>, Quantity<void, DimensionComponent<Length, 1>>>, Quantity<void, DimensionComponent<Length, 2>>>);

template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr Quantity<Engine_, DimensionComponents...>::Quantity(const Quantityland2::Quantity< OtherEngine, DimensionComponents... >& other)
    : m_value { detail::convert<Engine_>(other) }
{}

template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr Quantity< Engine_, DimensionComponents... >& Quantity<Engine_, DimensionComponents...>::operator= (const Quantityland2::Quantity< OtherEngine, DimensionComponents... >& other )
{
    m_value  = detail::convert<Engine> ( other );
    return *this;
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine, typename ...OtherDimensionComponents>
constexpr Quantity< Engine_, DimensionComponents... >& Quantity<Engine_, DimensionComponents...>::operator+= (const Quantityland2::Quantity< OtherEngine, OtherDimensionComponents... >& other)
{
    static_assert ( ( std::is_same_v<OtherDimensionComponents, DimensionComponents> && ... ), "Can't add quantities of differing dimension." );
    // We do the combination of broad template and static_assert to get better error messages.
    // Without the error message would be "no matching call to operator+ [...], candidates are: [List of 5 billion free operator+ overloads]"

    m_value += detail::convert<Engine> ( other );
    return *this;
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine, typename ...OtherDimensionComponents>
constexpr Quantity< Engine_, DimensionComponents... >& Quantity<Engine_, DimensionComponents...>::operator-= (const Quantityland2::Quantity< OtherEngine, OtherDimensionComponents... >& other)
{
    static_assert ( ( std::is_same_v<OtherDimensionComponents, DimensionComponents> && ... ), "Can't subtract quantities of differing dimension." );
    m_value -= detail::convert<Engine> ( other );
    return *this;
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine, typename ...OtherDimensionComponents>
constexpr Quantity< Engine_, DimensionComponents... > Quantity<Engine_, DimensionComponents...>::operator+ (const Quantityland2::Quantity<OtherEngine, OtherDimensionComponents...>& other)
{
    static_assert ( ( std::is_same_v<OtherDimensionComponents, DimensionComponents> && ... ), "Can't add quantities of differing dimension." );
    return { m_value + detail::convert<Engine> ( other ) };
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine, typename ...OtherDimensionComponents>
constexpr Quantity< Engine_, DimensionComponents... > Quantity<Engine_, DimensionComponents...>::operator- (const Quantityland2::Quantity<OtherEngine, OtherDimensionComponents...>& other )
{
    static_assert ( ( std::is_same_v<OtherDimensionComponents, DimensionComponents> && ... ), "Can't subtract quantities of differing dimension." );
    return { m_value - detail::convert<Engine> ( other ) };
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionComponents...>::operator== (const Quantityland2::Quantity<OtherEngine, DimensionComponents...>& other ) const
{
    return m_value == detail::convert<Engine> ( other );
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionComponents...>::operator!= (const Quantityland2::Quantity<OtherEngine, DimensionComponents...>& other ) const
{
    return m_value != detail::convert<Engine> ( other );
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionComponents...>::operator< (const Quantityland2::Quantity<OtherEngine, DimensionComponents...>& other ) const
{
    return m_value <  detail::convert<Engine> ( other );
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionComponents...>::operator> (const Quantityland2::Quantity<OtherEngine, DimensionComponents...>& other ) const
{
    return m_value >  detail::convert<Engine> ( other );
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionComponents...>::operator<= (const Quantityland2::Quantity<OtherEngine, DimensionComponents...>& other ) const
{
    return m_value <= detail::convert<Engine> ( other );
}


template<typename Engine_, typename ...DimensionComponents>
template<typename OtherEngine>
constexpr bool Quantity<Engine_, DimensionComponents...>::operator>= (const Quantityland2::Quantity<OtherEngine, DimensionComponents...>& other ) const
{
    return m_value >= detail::convert<Engine> ( other );
}


template<typename Engine1, typename Engine2,typename ...DimensionPack1, typename ...DimensionPack2>
constexpr auto operator*(const Quantity<Engine1, DimensionPack1...> &lhs, const Quantity<Engine2, DimensionPack2...> &rhs)
    -> detail::mergeMul_t<Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>>
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine1, DimensionPack1...>>,
                  "The left hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    static_assert(verifyDimensionOrder_v<Quantity<Engine2, DimensionPack2...>>,
                  "The right hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    using Ret_t = detail::mergeMul_t<Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() * detail::convert<Engine1>(rhs));
}

template<typename Engine1, typename Engine2,typename ...DimensionPack1, typename ...DimensionPack2>
constexpr auto operator/(const Quantity<Engine1, DimensionPack1...> &lhs, const Quantity<Engine2, DimensionPack2...> &rhs)
    -> detail::mergeDiv_t<Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>>
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine1, DimensionPack1...>>,
                  "The left hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    static_assert(verifyDimensionOrder_v<Quantity<Engine2, DimensionPack2...>>,
                  "The right hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    using Ret_t = detail::mergeDiv_t<Quantity<Engine1, DimensionPack1...>, Quantity<Engine2, DimensionPack2...>>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() / detail::convert<Engine1>(rhs));
}

template<typename Scalar, typename Engine, typename ...DimensionPack>
constexpr auto operator*(const Quantity<Engine, DimensionPack...> &lhs, Scalar rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs.numericalValue(), rhs), double>, Quantity<Engine, DimensionPack...>>
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine, DimensionPack...>>,
                  "The left hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    using Ret_t = Quantity<Engine, DimensionPack...>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() * rhs);
}

template<typename Scalar, typename Engine, typename ...DimensionPack>
constexpr auto operator*(Scalar lhs, const Quantity<Engine, DimensionPack...> &rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs * rhs.numericalValue()), double>, Quantity<Engine, DimensionPack...>>
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine, DimensionPack...>>,
                  "The right hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    using Ret_t = Quantity<Engine, DimensionPack...>;
    return detail::fromNumericalValue<Ret_t>(lhs * rhs.numericalValue());
}

template<typename Scalar, typename Engine, typename ...DimensionPack>
constexpr auto operator/(const Quantity<Engine, DimensionPack...> &lhs, Scalar rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs.numericalValue() / rhs), double>, Quantity<Engine, DimensionPack...>>
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine, DimensionPack...>>,
                  "The left hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    using Ret_t = Quantity<Engine, DimensionPack...>;
    return detail::fromNumericalValue<Ret_t>(lhs.numericalValue() / rhs);
}

template<typename Scalar, typename Engine, typename ...DimensionPack>
constexpr auto operator/(Scalar lhs, const Quantity<Engine, DimensionPack...> &rhs)
    -> std::enable_if_t<std::is_convertible_v<decltype(lhs / rhs.numericalValue()), double>, detail::invert_t<Quantity<Engine, DimensionPack...>>>
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine, DimensionPack...>>,
                  "The right hand operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    using Ret_t = detail::invert_t<Quantity<Engine, DimensionPack...>>;
    return detail::fromNumericalValue<Ret_t>(lhs / rhs.numericalValue());
}


template<int exp, typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
auto root(const Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...> &base)
{
    static_assert(verifyDimensionOrder_v<Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...>>,
                  "The operand doesn't obey the required ordre of base dimensions, e.g. Length MUST be listed before Mass.");
    static_assert(((exponentPack % exp == 0) && ...), "Extracting the root is not possible: Quantityland2 doesn't support rational exponents yet.");
    using Ret_t = Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack / exp>...>;
    if constexpr (exp == 2) {
        return detail::fromNumericalValue<Ret_t>(std::sqrt(base.numericalValue()));
    } else if constexpr (exp == 3) {
        return detail::fromNumericalValue<Ret_t>(std::cbrt(base.numericalValue()));
    } else {
        return detail::fromNumericalValue<Ret_t>(std::pow(base.numericalValue(), 1.0 / exp));
    }
}

template<typename Engine, typename ...BaseDimensionPack, int ...exponentPack>
std::ostream& operator<<(std::ostream& os, const Quantity<Engine, DimensionComponent<BaseDimensionPack, exponentPack>...>& v)
{
    if constexpr ((definesUnitString_v<BaseDimensionPack, Engine> && ...)) {
        os << v.numericalValue() << detail::formatUnit(v);
    } else if constexpr ((definesUnitString_v<BaseDimensionPack, typename Engine::referenceEngine> && ...)) {
        Quantity<typename Engine::referenceEngine, DimensionComponent<BaseDimensionPack, exponentPack>...> convertedV = v;
        os << convertedV.numericalValue() << detail::formatUnit(convertedV);
    } else {
        os << v.numericalValue() << " arb. unit";
    }
    return os;
}

}
