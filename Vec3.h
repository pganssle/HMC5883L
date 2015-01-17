/** @file
Definition for template class Vec3, which holds 3D Cartesian vectors.
*/

template<typename T> class Vec3 {
    /** Class for holding Cartesian 3-vectors.

    This is a class for holding Cartesian 3-vectors of values with type T (assumed to be numeric
    types). Addition, subtraction, division and multiplication are defined such that vector
    arithmetic is element-wise (e.g. `[1, 2, 3] * [2, 4, 7] == [2, 8, 21]`), and scalar operations
    are applied to all elements (e.g. `[1, 2, 3] + 4 == [5, 6, 7]`).
    
    */
    T x, y, z;

    Vec3<T>(T X, T Y, T Z) {
        x = X;
        y = Y;
        z = Z;
    }

    // Vec3-Vec3 operations
    Vec3<T> operator+(Vec3<T> v) {
        return Vec3<T>(x + v.x,
                       y + v.y,
                       z + v.z);
    }

    Vec3<T> operator-(Vec3<T> v) {
        return Vec3<T>(x - v.x,
                       y - v.y,
                       z - v.z);
    }

    Vec3<T> operator*(Vec3<T> v) {
        return Vec3<T>(x * v.x,
                       y * v.y,
                       z * v.z);
    }

    Vec3<T> operator/(Vec3<T> v) {
        return Vec3<T>(x / v.x,
                       y / v.y,
                       z / v.z);
    }

    // Vec3-int operations
    Vec3<T> operator+(int c) {
        return Vec3<T>(x + c,
                       y + c,
                       z + c);
    }

    Vec3<T> operator-(int c) {
        return Vec3<T>(x - c,
                       y - c,
                       z - c);
    }

    Vec3<T> operator*(int c) {
        return Vec3<T>(x * c,
                       y * c,
                       z * c);
    }

    Vec3<T> operator/(int c) {
        return Vec3<T>(x / c,
                       y / c,
                       z / c);
    }

    // Vec3-floating point operations
    Vec3<T> operator+(double c) {
        return Vec3<T>(x + c,
                       y + c,
                       z + c);
    }

    Vec3<T> operator-(double c) {
        return Vec3<T>(x - c,
                       y - c,
                       z - c);
    }

    Vec3<T> operator*(double c) {
        return Vec3<T>(x * c,
                       y * c,
                       z * c);
    }

    Vec3<T> operator/(double c) {
        return Vec3<T>(x / c,
                       y / c,
                       z / c);
    }
};