function OUT =modelonaolinear(IN)

Estados = IN(1:16); % estados dos sistema
dq = IN(9:16); % aceleraçoes das coordenadas generalizadas
%dq = dq';
U = IN(17:20); % sinais de controle
%U = U';
D = IN(21: 28); % Pertubaçoes em todas as acelaraçoes (Forças)
%D = D';

Mm = matrizM(Estados);
Cm = matrizC(Estados);
Gm = vetorG(Estados);
Bm = matrizB(Estados);

d2q = Mm\(Bm*U - Cm*dq - Gm + D); 

OUT = [dq;
       d2q];

end