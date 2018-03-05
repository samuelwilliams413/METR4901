syms t x(t) y(t) z a(t)
f3 = x^2+y^2+z^2+5+t;
xx = sym('x');
df3 = subs(diff(subs(f3,x,xx),xx),xx,x)

a1 = cos(a(t))
A = 'a'
aa = sym('a');

df = subs(int(subs(a1,a,aa),aa),aa,a)
ddf = subs(int(subs(df,a,aa),aa),aa,a)




integral = get_Integral(a1, A) 