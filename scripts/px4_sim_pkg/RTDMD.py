import numpy as np


class RTDMD():
    def __init__(self, delta, lam, aug, stateDim, inputDim):
        self.delta = delta
        self.lam = lam
        self.aug = aug
        self.stateDim = stateDim
        self.inputDim = inputDim
        self.stateInputDim = stateDim + inputDim
        self.augStateDim = aug * stateDim
        self.augInputDim = aug * inputDim
        self.augStateInputDim = aug * self.stateInputDim
        self.rtdmdResult = {
            'P': np.eye(self.augStateInputDim) / delta,
            'AB': np.zeros((aug * stateDim, self.augStateInputDim)),
            'A': np.zeros((aug * stateDim, aug * stateDim)),
            'B': np.zeros((aug * stateDim, aug * inputDim)),
            'g': np.zeros(self.augStateInputDim),
            'y_hat': np.zeros(stateDim),
            'error': np.zeros(aug * stateDim),
        }
        # return dmd


    def concatenate(self, data1, data2):
        return np.concatenate([data1, data2], axis=0)

    def create_delay_coordinates(self, x):
        """
        CREATE_DELAY_COORDINATES
        Description:
        Function to create delay coordinates
        Stack time series data
        Parameters:
        ----------
        x : array, Horizontal state
        aug : int, Number of columns in the delay coordinate system. This becomes aug * dimension of the data.

        Returns
        -------
        x_delay: array, Matrix with (aug * dimension of the data) rows and (length of x - aug + 1) columns
        """
        if self.aug > x.size:
            # Raise an exception if the number of elements is less than aug
            raise ValueError('aug must be less than the number of elements in x')

        if x.shape[0] > x.shape[1]:
            # Raise an exception if x is not horizontally long
            raise ValueError('rows of x must be less than the number of columns in x. You should use transpose of x')

        x_row = x.shape[0]
        x_col = x.shape[1]
        aug_dim = self.aug * x_row
        aug_col = x_col - self.aug + 1
        x_delay = np.zeros((aug_dim, aug_col))

        print(f"aug: {self.aug}")  # Number of extended rows per type of data
        print(f"aug dimension: {aug_dim} x {aug_col}")  # Number of rows and columns of the augmented matrix

        for i in range(aug_col):
            x_delay[:, i] = x[:, i:i+self.aug].reshape(aug_dim, order="F")

        return x_delay

    def predict_state(self, A, X, nTime):
        """
        Predicts state using A-matrix obtained by DMD.
        Parameters:
        A: A-matrix obtained by DMD.
        X: State delay coordinate.
        nTime: Length of time array.
        aug: Number of augmented dimensions of the delay coordinate system.
        stateDim: Dimension of the original state.
        Returns:
        xPredict: Predicted state by A-matrix.
        """
        nIteration = nTime - self.aug + 1
        xDmdStack = np.zeros((A.shape[0], nIteration))
        xDmdStack[:, 0] = X[:self.stateDim*self.aug, 0]  # Stores initial values

        # Predict state and stores predicted state
        for i in range(1, nIteration):
            xDmdStack[:, i] = np.dot(A, X[:, i-1])

        xPredict = np.zeros((self.stateDim, nTime))
        # Data for the first "aug" columns uses the initial data of the delayed coordinate X
        xPredict[:, :self.aug] = X[:self.stateDim, :self.aug]
        # Adjusting indexes for Python's 0-based indexing
        xPredict[:, self.aug:nTime] = xDmdStack[-self.stateDim:, 1:]

        return xPredict

    def calculate_fit_error(self, Actual, Predicted):
        """
        Calculates fit error between actual and predicted data.
        Parameters:
        Actual: Actual data matrix.
        Predicted: Predicted data matrix.
        Returns:
        E: Error metrics.
        """
        if Actual.shape != Predicted.shape:
            print('Size of Actual and Predicted are different')
            return None

        E = np.zeros((Actual.shape[0], 1))
        for i in range(Actual.shape[0]):
            y = Actual[i, :]
            yhat = Predicted[i, :]
            num = np.sum((yhat - y) ** 2)
            den = np.sum((y - np.mean(y)) ** 2)
            E[i] = (1 - np.sqrt(num) / np.sqrt(den)) * 100

        return E

    def update_rtdmd(self, XXUU, new_XXUU, count):
        y = new_XXUU[:self.augStateDim]
        P = self.rtdmdResult["P"]
        AB = self.rtdmdResult["AB"]

        y_hat_prior = AB @ XXUU
        error = y - y_hat_prior
        g = (P @ XXUU) / (self.lam + XXUU.T @ P @ XXUU)
        delta_W = error[:, np.newaxis] @ g[np.newaxis, :]
        AB += delta_W
        y_hat = y_hat_prior

        P = (P - g[:, np.newaxis] @ XXUU[np.newaxis, :] @ P) / self.lam

        P = 0.5 * (P + P.T)  # Make P symmetric
        # if self.isPLimit:
        #     if np.max(P) > self.PLimit:
        #         print("reset P at iteration", count)
        #         P = self.PLimit * np.eye(self.augStateInputDim)

        self.rtdmdResult = {
            'P': P,
            'AB': AB,
            'A': AB[:self.augStateDim, :self.augStateDim],
            'B': AB[:self.augStateDim, self.augStateDim:],
            'g': g,
            'y_hat': y_hat[-self.stateDim:],
            'error': error,
        }

        # return self.dmd
