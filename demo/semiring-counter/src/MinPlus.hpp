#include <boost/multiprecision/gmp.hpp>

namespace mpz = boost::multiprecision;

class MinPlus {
private:
    mpz::mpz_int value;
    bool is_inf; // represents -∞

public:
    // Default constructor = zero (-∞)
    MinPlus() : value(0), is_inf(true) {}

    // Constructor from mpz_int
  //MinPlus(const mpz::mpz_int& v) : value(v), is_inf(false)  {}
  
  MinPlus(const mpz::mpf_float& x) : value(static_cast<mpz::mpz_int>(floor(x))), is_inf(false) {}
  // Static constructors for zero and one
    static MinPlus zero() {
        return MinPlus(); // -∞
    }

    static MinPlus one() {
        return MinPlus(mpz::mpz_int(0)); // multiplicative identity
    }

    // Min (addition in semiring)
    MinPlus operator+(const MinPlus& other) const {
        if (is_inf) return other;
        if (other.is_inf) return *this;
        return MinPlus(value < other.value ? value : other.value);
    }

    // Plus (multiplication in semiring)
    MinPlus operator*(const MinPlus& other) const {
        if (is_inf || other.is_inf) return zero();
        return MinPlus(value + other.value);
    }
    MinPlus& operator*=(const MinPlus& other) {
      return *this = *this * other;

    }
    MinPlus& operator+=(const MinPlus& other) {
      return *this = *this + other;

    }

    bool operator==(const MinPlus& other) const {
        if (is_inf && other.is_inf) return true;
        if (is_inf || other.is_inf) return false;
        return value == other.value;
    }

  // Casting


  explicit operator mpz::mpz_int() const {
    return value; // beware inifinite values
  }
  explicit operator mpz::mpf_float() const {
    return value; // beware inifinite values
  }

explicit operator double() const {
    if (is_inf)
        return std::numeric_limits<double>::infinity();
    return static_cast<double>(value);
}

explicit operator float() const {
    if (is_inf)
        return std::numeric_limits<float>::infinity();
    return static_cast<float>(value);
}

explicit operator long double() const {
    if (is_inf)
        return std::numeric_limits<long double>::infinity();
    return static_cast<long double>(value);
}
  
    // Accessors
    bool isZero() const { return is_inf; }

    const mpz::mpz_int& getValue() const { return value; }
};


std::ostream& operator<<(std::ostream& os, const MinPlus& x) {
    if (x.isZero()) {
        return os << "+inf";
    }
    return os << x.getValue();
}
