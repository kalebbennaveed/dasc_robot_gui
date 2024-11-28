
#pragma once

#include <cmath>

// '''
// Header file for the lissa jous trajectory
// - Defines the lissa jous class with memeber functions
//   - set params to set the values of amplitude (A), the angular frequency (omega/w), phi, and the offset
//   - The equation has the form x = offset + A(wt + phi) 
// '''

template <typename T, int D> class Lissajous {

public:
  Lissajous(){};
  Lissajous(T amplitude[D], T omega[D], T phi[D], T offset[D]);

  void set_params(T amplitude[D], T omega[D], T phi[D], T offset[D]);

  void evaluate(T res[], T time_s, int derivative = 0);

private:
  T amplitude_[D];
  T omega_[D];
  T phi_[D];
  T offset_[D];
};

template <typename T, int D>
Lissajous<T, D>::Lissajous(T amplitude[D], T omega[D], T phi[D], T offset[D]) {

  set_params(amplitude, omega, phi, offset);
}

template <typename T, int D>
void Lissajous<T, D>::set_params(T amplitude[D], T omega[D], T phi[D],
                                 T offset[D]) {
  for (size_t i = 0; i < D; i++) {
    amplitude_[i] = amplitude[i];
    omega_[i] = omega[i];
    phi_[i] = phi[i];
    offset_[i] = offset[i];
  }
}

// d is the derivative 
template <typename T, int D>
void Lissajous<T, D>::evaluate(T res[], T time_s, int d) {

  if (d == 0) {
    for (size_t i = 0; i < D; i++) {
      res[i] =
          offset_[i] + amplitude_[i] * std::sin(omega_[i] * time_s + phi_[i]);
    }
    return;
  }

  for (size_t i = 0; i < D; i++) {
    res[i] = amplitude_[i] * std::pow(omega_[i], d) *
             std::sin(omega_[i] * time_s + 0.5 * M_PI * d + phi_[i]);
  }

  return;
}
