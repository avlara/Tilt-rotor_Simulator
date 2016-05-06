function G = vetorG(IN)

phi = IN(4);
theta=IN(5);
G=[0,0,0.194429E2,0.155481E-3.*cos(phi).*cos(theta)+0.928576E-1.*cos( ...
  theta).*sin(phi),0.132847E-4.*cos(theta)+0.928576E-1.*cos(phi).*sin( ...
  theta)+(-0.155481E-3).*sin(phi).*sin(theta),0,0,0];
G=G'

end