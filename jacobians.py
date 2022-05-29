# IMPORTS
from stat import S_ENFMT
import sympy as sym
from sympy import pprint, pi
from sympy.printing.octave import octave_code
from sympy.printing.latex import latex
from sympy.functions import *

# SETUP
sym.init_printing(use_unicode=True, wrap_line=False)

# DEFINE COMMONS
x_k = sym.Symbol('x_k', real=True)
y_k = sym.Symbol('y_k', real=True)
v_k = sym.Symbol('v_k', real=True)
t = sym.Symbol('t', real=True)
theta_k = sym.Symbol('theta_k', real=True)
omega_k = sym.Symbol('omega_k', real=True)

s_v_k = sym.Symbol('s_v_k', real=True)
s_omega_k = sym.Symbol('s_omega_k', real=True)

A = sym.Matrix([x_k + (v_k+s_v_k) * t * cos(theta_k), y_k + (v_k+s_v_k) * t * sin(theta_k), theta_k + (omega_k+s_omega_k) * t])
B = sym.Matrix([x_k, y_k, theta_k])
pprint(A)
print()

print('Jfx')
Jfx = A.jacobian(B)
pprint(Jfx.subs({s_v_k:0, s_omega_k:0}))
print(octave_code(Jfx))
print()

C = sym.Matrix([s_v_k, s_omega_k])

print('Jfw')
Jfw = A.jacobian(C)
pprint(Jfw.subs({s_v_k:0, s_omega_k:0}))
print(octave_code(Jfw))
print()









x_l = sym.Symbol('x_l', real=True)
y_l = sym.Symbol('y_l', real=True)
x_k1 = sym.Symbol('x_k1', real=True)
y_k1 = sym.Symbol('y_k1', real=True)
theta_k1 = sym.Symbol('theta_k1', real=True)
omega_r = sym.Symbol('omega_r', real=True)
omega_phi = sym.Symbol('omega_phi', real=True)

R = sqrt((x_l-x_k1)**2+(y_l-y_k1)**2)+omega_r
PHI = atan((y_l-y_k1)/(x_l-x_k1)) - theta_k1 + omega_phi

D = sym.Matrix([R, PHI])
E = sym.Matrix([x_k1, y_k1, theta_k1])

print('Jh')
Jh = D.jacobian(E)
pprint(Jh)
print(octave_code(Jh))


