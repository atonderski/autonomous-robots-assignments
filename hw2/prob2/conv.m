A = [
    3,3,2,1,8,8,9,9;
    3,2,2,7,8,9,9,10;
    3,3,3,7,7,8,9,10;
    3,3,4,6,6,8,8,9;
    4,4,5,7,7,8,8,10;
    5,5,6,8,8,9,9,10;
    5,5,6,8,8,9,9,10;
    7,6,7,9,9,9,9,10
    ];

K1 = [0,1,0;
      1,-5,1;
      0,1,0];
K2 = [1,0,-2;
      0,0,0;
      -2,0,0];
K3 = [0.1,0.2,0.1;
      0.3,0.5,0.3;
      0.1,0.3,0.2];
W  = [0.1,0.2,0.2,0.1,0.2,0.1,0.1,0.1];


L1 = conv2(A, K1, 'same');
% remove border since there was no zero padding
L1 = L1(2:7, 2:7)

L21 = conv2(L1, K2, 'same')
L22 = conv2(L1, K3, 'same')

% in maxpooling we 
L31 = [max(max(L21(1:3,1:3))), max(max(L21(1:3,4:6)));
       max(max(L21(4:6,1:3))), max(max(L21(4:6,4:6)))]
L32 = [max(max(L22(1:3,1:3))), max(max(L22(1:3,4:6)));
       max(max(L22(4:6,1:3))), max(max(L22(4:6,4:6)))]

flat = [L31(:); L32(:)]
out = W*flat