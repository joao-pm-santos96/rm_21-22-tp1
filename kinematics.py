# IMPORTS
import sympy as sym
from sympy import pprint, pi
from sympy.printing.octave import octave_code
from sympy.printing.latex import latex
from sympy.functions import *

# SETUP
sym.init_printing(use_unicode=True, wrap_line=False)

# DEFINE COMMONS
R = sym.Symbol('R', real=True, positive=True)
L = sym.Symbol('L', real=True, positive=True)
t = sym.Symbol('t', real=True, positive=True)
# dt = sym.Symbol('dt', real=True, positive=True)

X = sym.Symbol('x', real=True)
Y = sym.Symbol('y', real=True)
TH = sym.Symbol('theta', real=True)

X0 = sym.Symbol('x_0', real=True)
Y0 = sym.Symbol('y_0', real=True)
TH0 = sym.Symbol('theta_0', real=True)

theta = sym.Function('theta', real=True)(t)

dt = sym.Symbol('∆t', real=True, positive=True)
dx = sym.Symbol('∆x', real=True)
dy = sym.Symbol('∆y', real=True)
dth = sym.Symbol('∆theta', real=True)

Rot = sym.Matrix([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1]])
start_pos = sym.Matrix([[X0],[Y0],[TH0]])

# DEFINE FUNCTIONS
def DiffDrive():

    wl = sym.Symbol('omega_L', real=True)
    wr = sym.Symbol('omega_R', real=True)    

    # Forward
    M = sym.Matrix([[R/2, R/2],[0, 0],[-R/L, R/L]])    
    wheels = sym.Matrix([[wl],[wr]])
    vels = Rot*M*wheels  

    # Inverse    
    omega = vels[2]
    vels = vels.subs(theta, omega * t)
    frw_k = sym.integrate(vels, (t,0,dt)) + start_pos
    frw_k = sym.simplify(frw_k)

    inv_ks = []
    for i in [0, 1]:

        # pprint(pos[0].args[i].cond)

        A = sym.Matrix([[frw_k[0].args[i].expr],[frw_k[2]]])
        B = sym.Matrix([[X],[TH]])
        eq = sym.Eq(A, B)
        inv_k = sym.solve(eq, (wl, wr), dict=True)

        for sol in inv_k:
            inv_ks.append(sol)

    return(frw_k, inv_ks)      

def OmniDrive():

    w1 = sym.Symbol('omega_1', real=True)
    w2 = sym.Symbol('omega_2', real=True)
    w3 = sym.Symbol('omega_3', real=True)

    # Forward
    M = sym.Matrix([[0, R/sqrt(3), R/sqrt(3)],[-2*R/3, R/3, -R/3],[R/(3*L), R/(3*L), -R/(3*L)]])
    wheels = sym.Matrix([[w1],[w2],[w3]])

    vels = Rot*M*wheels
    vels = sym.simplify(vels)

    # Inverse
    omega = vels[2]
    vels = vels.subs(theta, omega * t)
    frw_k = sym.integrate(vels, (t,0,dt)) + start_pos
    frw_k = sym.simplify(frw_k)

    inv_ks = []
    for i in [1]:

        # pprint(pos[0].args[i].cond)

        A = sym.Matrix([[frw_k[0].args[i].expr],[frw_k[1].args[i].expr],[frw_k[2]]])
        B = sym.Matrix([[X],[Y],[TH]])
        eq = sym.Eq(A,B)

        inv_k = sym.solve(eq, (w1, w2, w3), dict=True)

        for sol in inv_k:
            inv_ks.append(sol)

    return(frw_k, inv_ks)

def Tricycle():
    
    w = sym.Symbol('omega', real=True)
    alpha = sym.Symbol('alpha', real=True)

    # Forward
    M = sym.Matrix([[1],[0],[tan(alpha)/L]]) * w * R

    vels = Rot * M
    vels = sym.simplify(vels)

    # Inverse
    omega = vels[2]
    vels = vels.subs(theta, omega * t)
    frw_k = sym.integrate(vels, (t,0,dt)) + start_pos
    frw_k = sym.simplify(frw_k)
 
    inv_ks = []
    for i in [0,1]:

        # pprint(pos[0].args[i].cond)

        A = sym.Matrix([[frw_k[0].args[i].expr],[frw_k[2]]])
        B = sym.Matrix([[X],[TH]])
        eq = sym.Eq(A, B)
        inv_k = sym.solve(eq, (w, alpha), dict=True)

        for sol in inv_k:
            inv_ks.append(sol)

    return(frw_k, inv_ks)

# MAIN
if __name__ == '__main__':

    for f in [DiffDrive, OmniDrive, Tricycle]:

        frw_k, inv_ks = f()
        
        print(f'{f.__name__}:')        
        
        pprint(frw_k)
        print(latex(frw_k))

        for sol in inv_ks:            
            for k in sol.keys():
                
                sol[k] =sym.separatevars(sol[k])
                
            pprint(sol)
            print(latex(sol))

        print('='*50)

