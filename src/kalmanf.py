# Kalman Filter Class

class KalmanF(object):
    """Class based Kalman Filter Methods"""
    def init(self, process_var, estimate_meas_var):
        """initialise"""
        self.Q = process_var # process variance
        self.R = estimate_meas_var # estimate of measurement variance
        # intial guesses
        self.xhat = 0.0  
        self.P = 1.0
    
    def input_noisy_meas(self, meas):
        """provide noisy measurement and perform
        time update -> measurement update
        to get updated extimates
        """
        # time update
        xhatminus = self.xhat
        Pminus = self.P + self.Q

        # measurement update
        K = Pminus/( Pminus + self.R )
        self.xhat = xhatminus + K * (meas - xhatminus)
        self.P = (1 - K) * Pminus
        
    def get_current_estimate(self):
        """method to get current measurement estiamte"""
        return self.xhat


if __name__ == "__main__":
    import numpy
    import pylab
    
    # intial parameters
    n_iter = 50
    sz = (n_iter,) # size of array
    x = -0.37727 # truth value (typo in example at top of p. 13 calls this z)
    noisy_meas = numpy.random.normal(x,0.1,size=sz) # observations (normal about x, sigma=0.1)
