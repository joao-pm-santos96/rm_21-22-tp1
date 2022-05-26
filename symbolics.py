# IMPORTS
import sympy as sym
from sympy import pprint
from sympy.printing.octave import octave_code
from sympy.functions import *

# SETUP
sym.init_printing(use_unicode=True, wrap_line=False)

# DEFINE COMMONS
R = sym.Symbol('R', real=True, positive=True)
L = sym.Symbol('L', real=True, positive=True)
t = sym.Symbol('t', real=True, positive=True)
tau = sym.Symbol('tau', real=True, positive=True)
X = sym.Symbol('X', real=True)
Y = sym.Symbol('Y', real=True)
TH = sym.Symbol('TH', real=True)

theta = sym.Function('theta', real=True)(t)

Rot = sym.Matrix([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1]])

def DiffDrive():

    wl = sym.Symbol('wl', real=True)
    wr = sym.Symbol('wr', real=True)    

    # Forward
    M = sym.Matrix([[R/2, R/2],[0, 0],[-R/L, R/L]])    
    wheels = sym.Matrix([[wl],[wr]])
    vels = Rot*M*wheels    

    # Inverse    
    omega = vels[2]
    vels = vels.subs(theta, omega * tau)
    pos = sym.integrate(vels, (tau,0,t))
    pos = sym.simplify(pos)

    pprint(pos)

    for i in [0,1]:

        pprint(pos[0].args[i].cond)

        A = sym.Matrix([[pos[0].args[i].expr],[pos[2]]])
        B = sym.Matrix([[X],[TH]])
        eq = sym.Eq(A, B)
        sol = sym.solve(eq, (wl, wr), dict=True)

        pprint(sol)

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
    vels = vels.subs(theta, omega * tau)
    pos = sym.integrate(vels, (tau,0,t))
    pos = sym.simplify(pos)

    pprint(pos)
    exit()
    
    for i in [1]:

        pprint(pos[0].args[i].cond)

        A = sym.Matrix([[pos[0].args[i].expr],[pos[1].args[i].expr],[pos[2]]])
        B = sym.Matrix([[X],[Y],[TH]])
        eq = sym.Eq(A,B)

        sol = sym.solve(eq, (w1, w2, w3), dict=True)

        pprint(sol)

def Tricycle():
    
    w = sym.Symbol('w', real=True)
    alpha = sym.Symbol('alpha', real=True)

    # Forward
    M = sym.Matrix([[1],[0],[tan(alpha)/L]]) * w * R
    # wheels = sym.Matrix([[w1],[w2]])

    vels = Rot * M
    vels = sym.simplify(vels)
    # pprint(vels)

    # Inverse
    omega = vels[2]
    vels = vels.subs(theta, omega * tau)
    pos = sym.integrate(vels, (tau,0,t))
    pos = sym.simplify(pos)

    pprint(pos)

    for i in [0,1]:

        pprint(pos[0].args[i].cond)

        A = sym.Matrix([[pos[0].args[i].expr],[pos[2]]])
        B = sym.Matrix([[X],[TH]])
        eq = sym.Eq(A, B)
        sol = sym.solve(eq, (w, alpha), dict=True)

        pprint(sol)



if __name__ == '__main__':
    
    # DiffDrive()
    # OmniDrive()
    Tricycle()
    
    # print(octave_code(sol))

