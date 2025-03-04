world { X:[0 0 .1] }

#thick radius floor and walls

floor (world){ shape:ssBox, Q:[0 0 -0.05], size:[4.1 4.1 .1 .04], color:[0.5 1 0], contact: 1 friction:10, logical:{table} }

outwall_right (world){ shape:ssBox, Q:[0 -2. 0.2], size:[4.1 .1 0.4 .04], color:[1 0 0], contact: 1 }
outwall_back (world){ shape:ssBox, Q:[2. 0 0.2], size:[.1 4.1 0.4 .04], color:[1 0 0], contact: 1 }
outwall_left (world){ shape:ssBox, Q:[0 2. 0.2], size:[4.1 .1 0.4 .04], color:[1 0 0], contact: 1 }
outwall_front (world){ shape:ssBox, Q:[-2. 0 0.2], size:[.1 4.1 0.4 .04] , color:[1 0 0], contact: 1 }


