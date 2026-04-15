#include <boost/multiprecision/gmp.hpp>

namespace mpz = boost::multiprecision;

class MaxPlus {
private:
    mpz::mpz_int value;
    bool is_neg_inf; // represents -∞

public:
    // Default constructor = zero (-∞)
    MaxPlus() : value(0), is_neg_inf(true) {}

    // Constructor from mpz_int
  //MaxPlus(const mpz::mpz_int& v) : value(v), is_neg_inf(false)  {}
  
  MaxPlus(const mpz::mpf_float& x) : value(static_cast<mpz::mpz_int>(floor(x))), is_neg_inf(false) {}
  // Static constructors for zero and one
    static MaxPlus zero() {
        return MaxPlus(); // -∞
    }

    static MaxPlus one() {
        return MaxPlus(mpz::mpz_int(0)); // multiplicative identity
    }

    // Max (addition in semiring)
    MaxPlus operator+(const MaxPlus& other) const {
        if (is_neg_inf) return other;
        if (other.is_neg_inf) return *this;
        return MaxPlus(value > other.value ? value : other.value);
    }

    // Plus (multiplication in semiring)
    MaxPlus operator*(const MaxPlus& other) const {
        if (is_neg_inf || other.is_neg_inf) return zero();
        return MaxPlus(value + other.value);
    }
    MaxPlus& operator*=(const MaxPlus& other) {
      return *this = *this * other;
    }
    MaxPlus& operator+=(const MaxPlus& other) {
      return *this = *this + other;
    }

    // Comparison (optional, useful)
    bool operator<(const MaxPlus& other) const {
        if (is_neg_inf) return !other.is_neg_inf;
        if (other.is_neg_inf) return false;
        return value < other.value;
    }

    bool operator==(const MaxPlus& other) const {
        if (is_neg_inf && other.is_neg_inf) return true;
        if (is_neg_inf || other.is_neg_inf) return false;
        return value == other.value;
    }

  bool operator==(const int& other) const {
        if (is_neg_inf) return false;
        return value == other;
    }

  // Casting


  explicit operator mpz::mpz_int() const {
    return value; // beware inifinite values
  }
  explicit operator mpz::mpf_float() const {
    return value; // beware inifinite values
  }

explicit operator double() const {
    if (is_neg_inf)
        return -std::numeric_limits<double>::infinity();
    return static_cast<double>(value);
}

explicit operator float() const {
    if (is_neg_inf)
        return -std::numeric_limits<float>::infinity();
    return static_cast<float>(value);
}

explicit operator long double() const {
    if (is_neg_inf)
        return -std::numeric_limits<long double>::infinity();
    return static_cast<long double>(value);
}
  
    // Accessors
    bool isZero() const { return is_neg_inf; }

    const mpz::mpz_int& getValue() const { return value; }
};


std::ostream& operator<<(std::ostream& os, const MaxPlus& x) {
    if (x.isZero()) {
        return os << "-inf";
    }
    return os << x.getValue();
}
