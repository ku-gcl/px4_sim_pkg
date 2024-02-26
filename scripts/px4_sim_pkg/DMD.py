import numpy as np


class DMD():
    # def __init__(self):
    def concatenate(self, data1, data2):
        return np.concatenate([data1, data2], axis=0)
    
    
    def splitdata(self, XX, stateDim=None, aug=None):
        """
        Description:
        Split delay coordinate data into X and XPrime.
        
        Parameters:
        XX: Delay coordinate data.
        stateDim: Dimension of state vector.
        aug: Number of columns of augmented matrix.
        
        Returns:
        X: Data from 1 to k.
        XPrime: Data from 2 to k+1.
        """
        self.X = XX[:, :-1]
        if stateDim is None and aug is None:
            # Exact DMD
            self.Xprime = XX[:, 1:]
        elif stateDim is not None and aug is not None:
            # DMDc: Exclude control inputs so that the A matrix is not a square matrix
            self.Xprime = XX[:stateDim*aug, 1:]
        else:
            raise ValueError("Number of input arguments should be 1 or 3")

    def DMD(self):
        """
        Compute dynamics model A using pseudo inverse.
        Parameters:
        X: Data at time "k".
        Xprime: Data at time "k+1".
        Returns:
        A: Dynamics model.
        """
        self.A = np.dot(self.Xprime, np.linalg.pinv(self.X))
        return self.A
    

    def predictstate(self, X):
        """
        Predicts state using A-matrix obtained by DMD.
        Parameters:
        A: A-matrix obtained by DMD.
        X: Augmented state.
        Returns:
        x_k1: Predicted state by A-matrix.
        """

        self.x_k1 = np.dot(self.A, X)
        return self.x_k1

