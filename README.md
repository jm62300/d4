# d4 with semirings

This fork of [**d4**](https://github.com/jm62300/d4) adds support for counting over semiring. The main changes can be found in the `demo/semiring` directory. Running `./build.sh` in this directory constructs the `semiring` executable which solves the algebraic model counting over the $(min, +)$-semiring. To change the semiring, one has to implement a new class with methods `one()` and `zero()` returning the identity of $\times$ and $+$ respectively. Moreover, the following operators must be overloaded (see the implementation from `demo/semiring/src/MinPlus.hpp` as an example):

- `+` and `+=`,
- `*` and `*=`,
- `==`,
- The std::ostream operator `<<`.

Moreover, the current architecture of the rest of the code of **d4** forces weights to be specified in the input as `mpz::mpf_float` (GMP floats) hence one needs to implement a constructor from this type and a casting to this type. This limitation will hopefully be solved soon in the main branch of **d4**. 

Once this class has been implemented, one can change the semiring used by the program in `demo/semiring/src/SemiringDemo.cpp` by replacing `semiringModel<mpz::mpz_int, MinPlus>` with `semiringModel<mpz::mpz_int, K>` where `K` is the desired semiring. 


