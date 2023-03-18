import numpy as np

class KalmanFilter:
    def __init__(self, F = None, G = None, H = None, Q = None, R = None, P = None, x0 = None):
        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F #State transition model
        self.H = H  #Observation matrix
        self.G = 0 if G is None else G #Control input model
        self.Q = np.eye(self.n) if Q is None else Q #Covariance process 
        self.R = np.eye(self.n) if R is None else R #Covariance observation
        self.P = np.eye(self.n) if P is None else P #Covariacne matrix
        self.x = np.zeros((self.n, 1)) if x0 is None else x0 #Init state


    def predict(self, u = 0,):
        #Control input
        U = np.dot(self.G, u)
        #State transition
        X = np.dot(self.F, self.x)
        #State extrapolation
        self.x = X + U
        #Covariance P_n+1 = F P_n_n F^T + Q
        FPFt = np.dot(np.dot(self.F, self.P), self.F.T)
        #Covariance extrapolaiton
        self.P = FPFt + self.Q
        return self.x  #Return the states

    def update(self, z, F = None, Q = None, G = None):
        #Update F,Q,G if we have non fixed step time
        self.F = self.F if F is None else self.F 
        self.Q = self.Q if Q is None else self.Q
        self.G = self.G if G is None else self.G
        #Diff between measurement and last state
        diff = z - np.dot(self.H, self.x)
        #print("Diff python: ", diff[0])
        #Gain calculation
        #P_n_n-1 H^T (H P_n_n-1 H^T + R)^-1
        HPHt = np.dot(self.H, np.dot(self.P, self.H.T))
        
        inv = np.linalg.inv(self.R + HPHt)
        #Gain = P_n_n-1 * H^T * inv
        K = np.dot(np.dot(self.P, self.H.T), inv)
        #State update 
        # X_n_n-1 + K diff
        #print("Gain1: ", K[0][0], " Gain2: ", K[1][0])
        self.x = self.x + np.dot(K, diff)
        #Covariance update
        # (I - K H) P_n_n-1 (I - K H)^T + K R K^T
        #Idenetity matrix
        I = np.eye(self.n)

        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

