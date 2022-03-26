package frc.robot.util.Interpolation;

public interface InverseInterpolable<T> {
    /**
     * Given this point (lower), a query point (query), and an upper point (upper), estimate how far (on [0, 1]) between
     * 'lower' and 'upper' the query point lies.
     *
     * @param upper
     * @param query
     * @return The interpolation parameter on [0, 1] representing how far between this point and the upper point the
     * query point lies.
     */
    double inverseInterpolate(T upper, T query);
}