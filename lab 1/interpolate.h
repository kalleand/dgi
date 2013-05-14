#ifndef KALLAND_EFAHLEN_INTERPOLATE
#define KALLAND_EFAHLEN_INTERPOLATE

#include <vector>

template<typename T>
void Interpolate(const T & a, const T & b, std::vector<T> & result) {
	// If we have one point or less, we need special treatement
	if (1 >= result.size()) {
		// If we should interpolate no point, do nothing
		if (result.size() == 0) return;

		// Only one point is needed
		result[0] = 0.5f * (a + b);
		return;
	}

	// Calculate how far every step will go
	T step = (b - a) * (1.0f / (result.size() - 1));

	// Setup the value to assign
	T value = a;

	// Loop through the result
	for (auto it = result.begin(); it != result.end(); ++it) {
		// Assign the value and increase it by the step
		*it = value;
		value += step;
	}
}

#endif
