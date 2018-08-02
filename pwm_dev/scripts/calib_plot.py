import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import least_squares

#def func(X, vs, ws):
#    cv, cw = X
#    v, w = vs*cv, ws*cw
#    return np.asarray([v,w])

def residual(p, x, y):
    vs, ws = p # param
    cv, cw, vv, vw = x # input
    v , w  = y # output
    print p
    return (1.0 / np.sqrt(vv))*np.abs(cv*vs-v) + (1.0 / np.sqrt(vw))*np.abs(cw*ws-w)

def jacobian(p, x, y):
    vs, ws = p # param
    cv, cw, vv, vw = x # input
    v , w  = y # output
    
    dc_dvs = 2.0*cv*(np.sqrt(vv)*np.abs(cw*ws - w) + np.sqrt(vw)*np.abs(cv*vs - v))*np.sign(cv*vs - v)/(vv*np.sqrt(vw))
    dc_dws = 2.0*cw*(np.sqrt(vv)*np.abs(cw*ws - w) + np.sqrt(vw)*np.abs(cv*vs - v))*np.sign(cw*ws - w)/(np.sqrt(vv)*vw)
    return np.stack((dc_dvs, dc_dws), axis=-1)

def main():
    M = np.load('/tmp/data.npy')
    # learn a mapping between
    # f(c_v,c_w) = v, w

    x = M[:,:2].T
    y = M[:,2:4].T
    w = M[:,4:].T # weight (variance), (Nx2)

    #plt.scatter(x[0], y[0])
    #plt.scatter(x[1], y[1])

    plt.plot(x[0])
    plt.plot(y[0])
    plt.plot(100 * w[0])
    plt.legend(['input','output', 'variance'])
    plt.show()

    #x = np.concatenate([x,w], axis=0) # merge with variance
    #print x.shape
    ## convert w to (N x (2x2))
    ##w = np.stack([w,w], axis=-1)
    ##w[:,0,1] = 0
    ##w[:,1,0] = 0

    print least_squares(residual, [0.9, 0.9], args=(x,y), jac=jacobian, loss='linear',
            xtol=1e-16
            )
    ##print w.shape
    ##print curve_fit(func, x, y, p0=(1,1))#, sigma=w)

if __name__ == "__main__":
    main()
