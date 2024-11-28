#include <iostream>

#include "Lissajous.hpp"

int main(int argc, char **argv) {

  std::cout << "hello\n";

  using T = float;
  constexpr int D = 3;

  T amplitude[D] = {1, 1, 1.};
  T omega[D] = {5, 5, 5.};
  T phi[D] = {0, 0, 0.};
  T offset[D] = {0, 0, 1.};

  Lissajous<T, D> lissa(amplitude, omega, phi, offset);

  for (float tau = 0; tau < 10; tau++) {
    T res[D];
    lissa.evaluate(res, tau, 0);

    std::cout << "tau: " << tau << ": ";
    for (size_t i = 0; i < D; i++) {
      std::cout << res[i] << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}
