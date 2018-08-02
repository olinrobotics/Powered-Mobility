from sympy import *

from sympy.assumptions.assume import global_assumptions

vv, cv, vs, v, vw, cw, ws, w = symbols('vv,cv,vs,v,vw,cw,ws,w', real=True)

#for x in [vv,cv,vs,v,vw,cw,ws,w]:
#    global_assumptions.add(Q.real(x))
cost = ((1.0 / sqrt(vv))*abs(cv*vs-v) + (1.0 / sqrt(vw))*abs(cw*ws-w))**2
print simplify(diff(cost, ws))
