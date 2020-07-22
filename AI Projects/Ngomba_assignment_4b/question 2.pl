add([], L, L).
add([X|L1], L2, [X|L3]) :-
   add(L1, L2, L3).
sbt(K,L,N) :- add(N,L,K).

mlt(_,[],[]).  
mlt(X, [_|L2], N) :-
   mlt(X, L2, L3),sbt(N,X,L3).

%dvd(L,N,K) :- mlt(K,N,L). formulate divide properly and use it for modulo


eq([],[]).
eq([_|T],[_|T1]) :- eq(T,T1). %equality


sub(S,L):- add(_,L2,L), add(S,_,L2),not(eq(S,L)).

dvd(K,L,[]):- sub(K,L). 
dvd(L1,N,[_|L2]) :- 
   dvd(L3,N,L2),sbt(L1,N,L3). 

mdl(K,L,K):- sub(K,L). 
mdl(L1,N,L2) :- 
   mdl(L3,N,L2),sbt(L1,N,L3).