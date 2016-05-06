function OUT = TiltFeedforward(IN)

dq = IN(9:16);
%dq = dq';
d2q =IN(17:24);% aceleraçoes das referências coordenadas generalizadas
%d2q = d2q';

Mm = matrizM(IN);
Cm = matrizC(IN);
Gm = vetorG(IN);
Bm = matrizB(IN);

OUT = Bm/(Mm*d2q + Cm*dq + Gm); 

end

