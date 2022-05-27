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
dt = sym.Symbol('dt', real=True, positive=True)

X = sym.Symbol('X', real=True)
Y = sym.Symbol('Y', real=True)
TH = sym.Symbol('TH', real=True)

X0 = sym.Symbol('X0', real=True)
Y0 = sym.Symbol('Y0', real=True)
TH0 = sym.Symbol('TH0', real=True)

theta = sym.Function('theta', real=True)(t)

Rot = sym.Matrix([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1]])
start_pos = sym.Matrix([[X0],[Y0],[TH0]])

# DEFINE FUNCTIONS
def DiffDrive():

    wl = sym.Symbol('wl', real=True)
    wr = sym.Symbol('wr', real=True)    

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

        inv_ks.append(inv_k)

    return(frw_k, inv_ks)      

def OmniDrive():

    w1 = sym.Symbol('w1', real=True)
    w2 = sym.Symbol('w2', real=True)
    w3 = sym.Symbol('w3', real=True)

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

        inv_ks.append(inv_k)

    return(frw_k, inv_ks)

def Tricycle():
    
    w = sym.Symbol('w', real=True)
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

        inv_ks.append(inv_k)

    return(frw_k, inv_ks)

# MAIN
if __name__ == '__main__':
    
    for f in [DiffDrive, OmniDrive, Tricycle]:

        frw_k, inv_ks = f()
        
        print(f'{f.__name__} :')

        # pprint(frw_k)

        for inv_k in inv_ks:
            print(octave_code(inv_k))
        
        # print(latex(inv_k))
        
        print()

