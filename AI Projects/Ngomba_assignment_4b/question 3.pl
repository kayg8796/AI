m(X,Y):-X=1,Y=1.
m(X,Y):-X = * ,Y=1.
m(X,Y):-X=0,Y=0.
m(X,Y):-X = * ,Y=0.

match([],[]).
match([K|L],[M|N]):-match(L,N),m(K,M).

matching([],_).
matching([X|L],A) :- matching(L,A) , match(X,A).